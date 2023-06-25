///////////////////////////////////////////////////////////////////////////////
// Hungarian.cpp: Implementation file for Class HungarianAlgorithm.
// 
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
// 
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
// 

#include "hungarian_algorithm.h"

#include <memory.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>

#ifdef NDEBUG
#define jmAssert(o) (void)0
#else
#define jmAssert(o) do {if (!(o)) *((int*)0) = 0;} while (0)
#endif

typedef struct HungarianAlgorithm {
	int *assignment;
	float* innerDistMatrix;
	int nRows;
	int nCols;
	BOOL *coveredColumns;
	BOOL *coveredRows;
	BOOL *starMatrix;
	BOOL *newStarMatrix;
	BOOL *primeMatrix;
}HungarianAlgorithm;

static float _assignmentoptimal(HungarianAlgorithm* o, const float* distMatrix);
static void _buildassignmentvector(HungarianAlgorithm* o);
static float _computeassignmentcost(HungarianAlgorithm* o, const float *distMatrix);
static void _step2a(HungarianAlgorithm* o, int minDim);
static void _step2b(HungarianAlgorithm* o, int minDim);
static void _step3(HungarianAlgorithm* o, int minDim);
static void _step4(HungarianAlgorithm* o, int minDim, int row, int col);
static void _step5(HungarianAlgorithm* o, int minDim);

int HungarianAlgorithm_getBufferSize(int nRows, int nCols)
{
	return sizeof(float) * nRows * nCols	// innerDistMatrix
		+ sizeof(BOOL) * (nCols + nRows		// coveredColumns + coveredRows
			+ 3 * nRows * nCols);			// starMatrix, primeMatrix, newStarMatrix
}

void* HungarianAlgorithm_solveWithBuffer(const float* distMatrix, int nRows, int nCols, int* assignment, float* cost, void* buffer)
{
	int i, j, nOfElements = nRows * nCols;
	float tmpCost = 0;
	HungarianAlgorithm hun;
	hun.nRows = nRows;
	hun.nCols = nCols;
	hun.assignment = assignment;
	for (i = 0; i < nRows; i++)
		hun.assignment[i] = -1;

	hun.innerDistMatrix = (float*)buffer;
	buffer = (void*)(hun.innerDistMatrix + nOfElements);

	/* check if all matrix elements are positive */
	/* generate working copy of distance Matrix with transpose, for matlab columns first */
	for (i = 0; i < nRows; i++)
		for (j = 0; j < nCols; j++) {
			jmAssert(distMatrix[i * nCols + j] >= 0);
			hun.innerDistMatrix[i + nRows * j] = distMatrix[i * nCols + j];
		}

	memset(buffer, 0, sizeof(BOOL) * (hun.nRows + hun.nCols + 3 * nOfElements));
	hun.coveredColumns = (BOOL*)buffer;						// (BOOL *)calloc(nCols, sizeof(BOOL));
	hun.coveredRows   = hun.coveredColumns + hun.nCols;		// (BOOL *)calloc(nRows, sizeof(BOOL));
	hun.starMatrix    = hun.coveredRows    + hun.nRows;		// (BOOL *)calloc(nOfElements, sizeof(BOOL));
	hun.primeMatrix   = hun.starMatrix     + nOfElements;	// (BOOL *)calloc(nOfElements, sizeof(BOOL));
	hun.newStarMatrix = hun.primeMatrix    + nOfElements;	// (BOOL *)calloc(nOfElements, sizeof(BOOL)); /* used in step4 */
	buffer     = (void*)(hun.newStarMatrix + nOfElements);	

	tmpCost = _assignmentoptimal(&hun, distMatrix);

	if (cost != NULL)
		*cost = tmpCost;
	return buffer;
}

BOOL HungarianAlgorithm_solve(const float* distMatrix, int nRows, int nCols, int* assignment, float* cost)
{
	int bufferSize = HungarianAlgorithm_getBufferSize(nRows, nCols);
	void* buffer = malloc(bufferSize), *bufferNext;
	if (buffer == NULL)
		return FALSE;
	bufferNext = HungarianAlgorithm_solveWithBuffer(distMatrix, nRows, nCols, assignment, cost, buffer);
	jmAssert((size_t)bufferNext - (size_t)buffer == bufferSize);
	free(buffer);
	return TRUE;
}

//********************************************************//
// Solve optimal solution for assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
//********************************************************//
 static float _assignmentoptimal(HungarianAlgorithm* o, const float* distMatrix)
{
	int nRows = o->nRows, nCols = o->nCols, row, col, minDim;
	float *innerDistMatrix = o->innerDistMatrix, *innerDistMatrixEnd = o->innerDistMatrix + nRows * nCols;
	float *columnEnd, *distMatrixTemp, value, minValue;

	/* preliminary steps */
	if (nRows <= nCols)
	{
		minDim = nRows;

		for (row = 0; row<nRows; row++)
		{
			/* find the smallest element in the row */
			distMatrixTemp = innerDistMatrix + row;
			minValue = *distMatrixTemp;
			distMatrixTemp += nRows;
			while (distMatrixTemp < innerDistMatrixEnd)
			{
				value = *distMatrixTemp;
				if (value < minValue)
					minValue = value;
				distMatrixTemp += nRows;
			}

			/* subtract the smallest element from each element of the row */
			distMatrixTemp = innerDistMatrix + row;
			while (distMatrixTemp < innerDistMatrixEnd)
			{
				*distMatrixTemp -= minValue;
				distMatrixTemp += nRows;
			}
		}

		/* Steps 1 and 2a */
		for (row = 0; row<nRows; row++)
			for (col = 0; col<nCols; col++)
				if (ABS(innerDistMatrix[row + nRows*col]) < FLT_EPSILON)
					if (!o->coveredColumns[col])
					{
						o->starMatrix[row + nRows*col] = TRUE;
						o->coveredColumns[col] = TRUE;
						break;
					}
	}
	else /* if(nOfRows > nOfColumns) */
	{
		minDim = nCols;

		for (col = 0; col<nCols; col++)
		{
			/* find the smallest element in the column */
			distMatrixTemp = innerDistMatrix + nRows*col;
			columnEnd = distMatrixTemp + nRows;

			minValue = *distMatrixTemp++;
			while (distMatrixTemp < columnEnd)
			{
				value = *distMatrixTemp++;
				if (value < minValue)
					minValue = value;
			}

			/* subtract the smallest element from each element of the column */
			distMatrixTemp = innerDistMatrix + nRows*col;
			while (distMatrixTemp < columnEnd)
				*distMatrixTemp++ -= minValue;
		}

		/* Steps 1 and 2a */
		for (col = 0; col<nCols; col++)
			for (row = 0; row<nRows; row++)
				if (ABS(innerDistMatrix[row + nRows*col]) < FLT_EPSILON)
					if (!o->coveredRows[row])
					{
						o->starMatrix[row + nRows*col] = TRUE;
						o->coveredColumns[col] = TRUE;
						o->coveredRows[row] = TRUE;
						break;
					}
		for (row = 0; row<nRows; row++)
			o->coveredRows[row] = FALSE;
	}

	/* move to step 2b */
	_step2b(o, minDim);

	/* compute cost and remove invalid assignments */
	return _computeassignmentcost(o, distMatrix);
}

/********************************************************/
static void _buildassignmentvector(HungarianAlgorithm* o)
{
	int row, col;
	for (row = 0; row < o->nRows; row++)
		for (col = 0; col < o->nCols; col++)
			if (o->starMatrix[row + o->nRows * col]) {
				o->assignment[row] = col;
				break;
			}
}

/********************************************************/
static float _computeassignmentcost(HungarianAlgorithm* o, const float *distMatrix)
{
	int row, col;
	float cost = 0;
	for (row = 0; row < o->nRows; row++)
	{
		col = o->assignment[row];
		if (col >= 0)
			cost += distMatrix[o->nCols * row + col];
	}
	return cost;
}

/********************************************************/
static void _step2a(HungarianAlgorithm* o, int minDim)
{
	BOOL *starMatrixTemp, *columnEnd;
	int col;

	/* cover every column containing a starred zero */
	for (col = 0; col < o->nCols; col++)
	{
		starMatrixTemp = o->starMatrix + o->nRows * col;
		columnEnd = starMatrixTemp + o->nRows;
		while (starMatrixTemp < columnEnd){
			if (*starMatrixTemp++)
			{
				o->coveredColumns[col] = TRUE;
				break;
			}
		}
	}

	/* move to step 3 */
	_step2b(o, minDim);
}

/********************************************************/
static void _step2b(HungarianAlgorithm* o, int minDim)
{
	int col, nOfCoveredColumns;

	/* count covered columns */
	nOfCoveredColumns = 0;
	for (col = 0; col< o->nCols; col++)
		if (o->coveredColumns[col])
			nOfCoveredColumns++;

	if (nOfCoveredColumns == minDim)
	{
		/* algorithm finished */
		_buildassignmentvector(o);
	}
	else
	{
		/* move to step 3 */
		_step3(o, minDim);
	}
}

/********************************************************/
static void _step3(HungarianAlgorithm* o, int minDim)
{
	BOOL zerosFound;
	int row, col, starCol;

	zerosFound = TRUE;
	while (zerosFound)
	{
		zerosFound = FALSE;
		for (col = 0; col < o->nCols; col++)
			if (!o->coveredColumns[col])
				for (row = 0; row < o->nRows; row++)
					if ((!o->coveredRows[row]) && (ABS(o->innerDistMatrix[row + o->nRows * col]) < FLT_EPSILON))
					{
						/* prime zero */
						o->primeMatrix[row + o->nRows * col] = TRUE;

						/* find starred zero in current row */
						for (starCol = 0; starCol < o->nCols; starCol++)
							if (o->starMatrix[row + o->nRows * starCol])
								break;

						if (starCol == o->nCols) /* no starred zero found */
						{
							/* move to step 4 */
							_step4(o, minDim, row, col);
							return;
						}
						else
						{
							o->coveredRows[row] = TRUE;
							o->coveredColumns[starCol] = FALSE;
							zerosFound = TRUE;
							break;
						}
					}
	}

	/* move to step 5 */
	_step5(o, minDim);
}

/********************************************************/
static void _step4(HungarianAlgorithm* o, int minDim, int row, int col)
{
	int n, starRow, starCol, primeRow, primeCol;
	int nRows = o->nRows, nOfElements = o->nRows * o->nCols;
	BOOL *starMatrix = o->starMatrix, *newStarMatrix = o->newStarMatrix;

	/* generate temporary copy of starMatrix */
	for (n = 0; n < nOfElements; n++)
		newStarMatrix[n] = starMatrix[n];

	/* star current zero */
	newStarMatrix[row + nRows * col] = TRUE;

	/* find starred zero in current column */
	starCol = col;
	for (starRow = 0; starRow < nRows; starRow++)
		if (starMatrix[starRow + nRows * starCol])
			break;

	while (starRow < nRows)
	{
		/* unstar the starred zero */
		newStarMatrix[starRow + nRows * starCol] = FALSE;

		/* find primed zero in current row */
		primeRow = starRow;
		for (primeCol = 0; primeCol < o->nCols; primeCol++)
			if (o->primeMatrix[primeRow + nRows * primeCol])
				break;

		/* star the primed zero */
		newStarMatrix[primeRow + nRows * primeCol] = TRUE;

		/* find starred zero in current column */
		starCol = primeCol;
		for (starRow = 0; starRow < nRows; starRow++)
			if (starMatrix[starRow + nRows * starCol])
				break;
	}

	/* use temporary copy as new starMatrix */
	/* delete all primes, uncover all rows */
	for (n = 0; n < nOfElements; n++)
	{
		o->primeMatrix[n] = FALSE;
		starMatrix[n] = newStarMatrix[n];
	}
	for (n = 0; n< nRows; n++)
		o->coveredRows[n] = FALSE;

	/* move to step 2a */
	_step2a(o, minDim);
}

/********************************************************/
static void _step5(HungarianAlgorithm* o, int minDim)
{
	float h, value, *innerDistMatrix = o->innerDistMatrix;
	int row, col, nRows = o->nRows, nCols = o->nCols;

	/* find smallest uncovered element h */
	h = FLT_MAX;
	for (row = 0; row < nRows; row++)
		if (!o->coveredRows[row])
			for (col = 0; col < nCols; col++)
				if (!o->coveredColumns[col])
				{
					value = innerDistMatrix[row + nRows * col];
					if (value < h)
						h = value;
				}

	/* add h to each covered row */
	for (row = 0; row < nRows; row++)
		if (o->coveredRows[row])
			for (col = 0; col < nCols; col++)
				innerDistMatrix[row + nRows * col] += h;

	/* subtract h from each uncovered column */
	for (col = 0; col < nCols; col++)
		if (!o->coveredColumns[col])
			for (row = 0; row < nRows; row++)
				innerDistMatrix[row + nRows * col] -= h;

	/* move to step 3 */
	_step3(o, minDim);
}
