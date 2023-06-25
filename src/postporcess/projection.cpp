#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ccalib/omnidir.hpp>

#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

//#include <sys/types.h>
#include "comm_api.h"
#include "projection.h"

#define SHOW_IN_PICTURE 1

float norm_value(std::vector<float> vector, int size) {
    int i;
    float square_sum = 0.;
    for (i = 0; i < size; i++) {
        square_sum += vector[i] * vector[i];
    }
    return sqrt(square_sum);
}

void vec_norm(std::vector<float> vector, std::vector<float>& vector_new, int size) {
    /*
    Note: 'vector' and 'vector_new' must have the same size.
    */
    int i;
    float square_sum = 0.;
    for (i = 0; i < size; i++) {
        square_sum += vector[i] * vector[i];
    }
    square_sum = sqrt(square_sum);
    if (square_sum < 1.0E-7)    square_sum = 1.0E-7;
    for (i = 0; i < size; i++) {
        vector_new[i] = vector[i] / square_sum;
    }
}

void get_quat_norm(std::vector<float> quat, std::vector<float>& quat_norm) {
    float n = norm_value(quat, 4);
    if (n > 0)  vec_norm(quat, quat_norm, 4);
}

void quat_to_rotMatrix_scipy(std::vector<float>& quat, std::vector<std::vector<float>>& rotMatrix)
{
    
    std::vector<float> quat_norm(4);
    get_quat_norm(quat, quat_norm);
    float x = quat_norm[0];
    float y = quat_norm[1];
    float z = quat_norm[2];
    float w = quat_norm[3];
    float x2 = x * x;
    float y2 = y * y;
    float z2 = z * z;
    float w2 = w * w;
    float xy = x * y;
    float zw = z * w;
    float xz = x * z;
    float yw = y * w;
    float yz = y * z;
    float xw = x * w;
    rotMatrix[0][0] = x2 - y2 - z2 + w2;
    rotMatrix[1][0] = 2 * (xy + zw);
    rotMatrix[2][0] = 2 * (xz - yw);
    rotMatrix[0][1] = 2 * (xy - zw);
    rotMatrix[1][1] = -x2 + y2 - z2 + w2;
    rotMatrix[2][1] = 2 * (yz + xw);
    rotMatrix[0][2] = 2 * (xz + yw);
    rotMatrix[1][2] = 2 * (yz - xw);
    rotMatrix[2][2] = -x2 - y2 + z2 + w2;
}
/*
cv::Mat projection_quanxiang(std::string imgname, torch::Tensor centers, torch::Tensor rot, torch::Tensor whls,int ii, torch::Tensor labels_3d, std::string cams_)
{
   



    for (int ll = 0; ll < centers.sizes()[0]; ll++)
    {
        torch::Tensor center = centers[ll];
        std::vector<float> quat(4);
        std::vector<float> r;
        std::vector<std::vector<float>> rotz(3, std::vector<float>(3));
        std::vector<std::vector<float>> rotz_new(3, std::vector<float>(3));
        quat[0] = 0;
        quat[1] = 0;
        quat[2] = sin(rot[ll].item().toFloat() / 2);
        quat[3] = cos(rot[ll].item().toFloat() / 2);
        quat_to_rotMatrix_scipy(quat, rotz);
        torch::Tensor whl = whls[ll];


        center1 = np.dot(ro, center1)  
       
         center1 += np.array(tr)
         if center1[-1] < 0 :
        continue

        // ���Գ�����ϵתΪ�������ϵ

        for (int i = 0; i < center.size(); i++)
        {
            center[i] -= tran.ptr<float>(i)[0];
        }
        std::vector<std::vector<float>> center_(3, std::vector<float>(1));
        std::vector<std::vector<float>> center_new(3, std::vector<float>(1));
        for (int i = 0; i < center.size(); i++)
        {
            center_[i][0] = center[i];
        }
        matrix_mul(rot_inv, 3, 3, center_, 1, 3, center_new);                      // �Գ�����ϵ��box��ת���������ϵ
        matrix_mul(rot_inv, 3, 3, rotz, 3, 3, rotz_new);

        if (center_new[2][0] < 0)    continue;

        // �����Գ�����ϵ�İ˸�����
        float w, h, l;
        w = whl[0];
        l = whl[1];
        h = whl[2];
        std::vector<float> x_corners = { 1,  1,  1,  1, -1, -1, -1, -1 };
        std::vector<float> y_corners = { 1, -1, -1,  1,  1, -1, -1,  1 };
        std::vector<float> z_corners = { 1,  1, -1, -1,  1,  1, -1, -1 };
        for (int i = 0; i < 8; i++) {
            x_corners[i] *= l / 2;
            y_corners[i] *= w / 2;
            z_corners[i] *= h / 2;
        }
        std::vector<std::vector<float>> corners;
        std::vector<std::vector<float>> corners_new(3, std::vector<float>(8));
        corners.push_back(x_corners);
        corners.push_back(y_corners);
        corners.push_back(z_corners);                             // ����8����
        matrix_mul(rotz_new, 3, 3, corners, 8, 3, corners_new);   // 8���������ת��
        for (int i = 0; i < center.size(); i++) {
            for (int j = 0; j < corners[i].size(); j++) {
                corners_new[i][j] += center_new[i][0];                   // 8����ƽ�����������ϵ
            }
        }
        // ����ͼ������ϵ�İ˸���������ĵ�����
        std::vector<std::vector<float>> corners_2d_homo(3, std::vector<float>(8));
        matrix_mul(intrin_, 3, 3, corners_new, 8, 3, corners_2d_homo);
        for (int i = 0; i < 8; i++) {
            corners_2d_homo[0][i] /= corners_2d_homo[2][i];
            corners_2d_homo[1][i] /= corners_2d_homo[2][i];
            corners_2d_homo[2][i] /= corners_2d_homo[2][i];       // ��λ�
        }
        int nn = 0;
        for (int g = 0; g < 8; g++)
        {
            if (corners_2d_homo[0][g] > 0 && corners_2d_homo[0][g] < 1600 && corners_2d_homo[1][g] > 0 && corners_2d_homo[1][g] < 960)
            {
                nn += 1;
                if (nn >= 2)
                {
                    for (int i = 0; i < 4; i++) {
                        cv::line(frame, cv::Point((int)(corners_2d_homo[0][i]), (int)(corners_2d_homo[1][i])), cv::Point((int)(corners_2d_homo[0][i + 4]), (int)(corners_2d_homo[1][i + 4])), cv::Scalar(0, 0, 255), 2);
                    }
                    // get head and tail corners
                    for (int id = 0; id < 4; id++) {
                        corners_2d_homo_head4[id][0] = corners_2d_homo[0][id];
                        corners_2d_homo_head4[id][1] = corners_2d_homo[1][id];
                        corners_2d_homo_head4[id][2] = corners_2d_homo[2][id];
                        corners_2d_homo_tail4[id][0] = corners_2d_homo[0][id + 4];
                        corners_2d_homo_tail4[id][1] = corners_2d_homo[1][id + 4];
                        corners_2d_homo_tail4[id][2] = corners_2d_homo[2][id + 4];
                    }
                    draw_cv(frame, corners_2d_homo_head4);
                    draw_cv(frame, corners_2d_homo_tail4);
                    std::vector<float> center_bottom_forward = get_center_bottom_forward(corners_2d_homo);
                    std::vector<float> center_bottom = get_center_bottom(corners_2d_homo);
                    cv::line(frame, cv::Point((int)(center_bottom[0]), (int)(center_bottom[1])),
                        cv::Point((int)(center_bottom_forward[0]), (int)(center_bottom_forward[1])), cv::Scalar(0, 0, 255), 2);
                }
            }
        }
    }
    cv::resize(frame, frame, cv::Size(800, 450));
    std::string img_save_dir = "D:/AI3DBUSHU/BEV_Det_cpp//BEV_Det_cpp/img_results/res_imgs/";
    if (view_id == 0) {
        std::string view_dir = "CAM_FRONT_LEFT/";
        std::string imgs_str_id = std::to_string(imgs_id);
        std::string imgs_str_id_withzero = name_string_same_length(imgs_str_id, 4);
        std::string img_file_path = img_save_dir + view_dir + imgs_str_id_withzero + ".jpg";
        // cv::imwrite(img_file_path, frame);
    }
    else if (view_id == 1) {
        std::string view_dir = "CAM_FRONT/";
        std::string imgs_str_id = std::to_string(imgs_id);
        std::string imgs_str_id_withzero = name_string_same_length(imgs_str_id, 4);
        std::string img_file_path = img_save_dir + view_dir + imgs_str_id_withzero + ".jpg";
        // cv::imwrite(img_file_path, frame);
    }
    else if (view_id == 2) {
        std::string view_dir = "CAM_FRONT_RIGHT/";
        std::string imgs_str_id = std::to_string(imgs_id);
        std::string imgs_str_id_withzero = name_string_same_length(imgs_str_id, 4);
        std::string img_file_path = img_save_dir + view_dir + imgs_str_id_withzero + ".jpg";
        // cv::imwrite(img_file_path, frame);
    }
    else if (view_id == 3) {
        std::string view_dir = "CAM_BACK_LEFT/";
        std::string imgs_str_id = std::to_string(imgs_id);
        std::string imgs_str_id_withzero = name_string_same_length(imgs_str_id, 4);
        std::string img_file_path = img_save_dir + view_dir + imgs_str_id_withzero + ".jpg";
        // cv::imwrite(img_file_path, frame);
    }
    else if (view_id == 4) {
        std::string view_dir = "CAM_BACK/";
        std::string imgs_str_id = std::to_string(imgs_id);
        std::string imgs_str_id_withzero = name_string_same_length(imgs_str_id, 4);
        std::string img_file_path = img_save_dir + view_dir + imgs_str_id_withzero + ".jpg";
        // cv::imwrite(img_file_path, frame);
    }
    else if (view_id == 5) {
        std::string view_dir = "CAM_BACK_RIGHT/";
        std::string imgs_str_id = std::to_string(imgs_id);
        std::string imgs_str_id_withzero = name_string_same_length(imgs_str_id, 4);
        std::string img_file_path = img_save_dir + view_dir + imgs_str_id_withzero + ".jpg";
        // cv::imwrite(img_file_path, frame);
    }

}
*/
void Mat_to_vec(cv::Mat rot, std::vector<std::vector<float>>& rot_)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot_[i][j] = rot.ptr<float>(i)[j];
        }
    }
}

void matrix_inverse(std::vector<std::vector<float>> a, int N, std::vector<std::vector<float>>& b)
{

    using namespace std;
    int i, j, k;
    double max, temp;
    double t[3][3];
    // ��a������ʱ����ھ���t[n][n]��
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            t[i][j] = a[i][j];
        }
    }
    // ��ʼ��B����Ϊ��λ����
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            b[i][j] = (i == j) ? (double)1 : 0;
        }
    }
    // ����������Ԫ���ҵ�ÿһ�е���Ԫ
    for (i = 0; i < N; i++)
    {
        max = t[i][i];
        // ���ڼ�¼ÿһ���еĵڼ���Ԫ��Ϊ��Ԫ
        k = i;
        // Ѱ��ÿһ���е���ԪԪ��
        for (j = i + 1; j < N; j++)
        {
            if (fabs(t[j][i]) > fabs(max))
            {
                max = t[j][i];
                k = j;
            }
        }
        //cout<<"the max number is "<<max<<endl;
        // �����Ԫ���ڵ��в��ǵ�i�У�������н���
        if (k != i)
        {
            // �����н���
            for (j = 0; j < N; j++)
            {
                temp = t[i][j];
                t[i][j] = t[k][j];
                t[k][j] = temp;
                // �������BҲҪ�����н���
                temp = b[i][j];
                b[i][j] = b[k][j];
                b[k][j] = temp;
            }
        }
        if (t[i][i] == 0)
        {
            cout << "\nthe matrix does not exist inverse matrix\n";
            break;
        }
        // ��ȡ����Ԫ��
        temp = t[i][i];
        // ����Ԫ���ڵ��н��е�λ������
        //cout<<"\nthe temp is "<<temp<<endl;
        for (j = 0; j < N; j++)
        {
            t[i][j] = t[i][j] / temp;
            b[i][j] = b[i][j] / temp;
        }
        for (j = 0; j < N; j++)
        {
            if (j != i)
            {
                temp = t[j][i];
                //��ȥ���е�����Ԫ��
                for (k = 0; k < N; k++)
                {
                    t[j][k] = t[j][k] - temp * t[i][k];
                    b[j][k] = b[j][k] - temp * b[i][k];
                }
            }
        }
    }
}
void matrix_mul(std::vector<std::vector<float>> A, int num_x_A, int num_y_A,
    std::vector<std::vector<float>> B, int num_x_B, int num_y_B,
    std::vector<std::vector<float>>& C) {
    /*
    Note:
    1. 'num_x_A' must be equal to 'num_y_B';
    2. the size of 'float* C' is 'num_y_A*num_xB', that is 'float C[num_y_A * num_x_B];'.
    */
    int i, j, m;
    for (j = 0; j < num_y_A; j++) {          // j is row
        for (i = 0; i < num_x_B; i++) {       // i is col
            float sum_tmp = 0.;
            for (m = 0; m < num_x_A; m++) {
                sum_tmp += A[j][m] * B[m][i];
            }
            C[j][i] = sum_tmp;
        }
    }
}

void draw_cv(cv::Mat& frame, std::vector<std::vector<float>> selected_corners)
{
    int n = selected_corners.size();
    std::vector<float> prev = selected_corners[n - 1];
    for (int i = 0; i < n; i++) {
        std::vector<float> corner = selected_corners[i];
        cv::line(frame, cv::Point(round(prev[0]), round(prev[1])), cv::Point(round(corner[0]), round(corner[1])), cv::Scalar(0, 0, 255), 3);
        prev[0] = corner[0];
        prev[1] = corner[1];
    }
}
std::vector<float> get_center_bottom_forward(std::vector<std::vector<float>> corners_2d_homo)
{
    std::vector<float> center_bottom_forward(2);
    for (int i = 0; i < 2; i++) {
        center_bottom_forward[i] = (corners_2d_homo[i][2] + corners_2d_homo[i][3]) / 2;
    }
    return center_bottom_forward;
}

std::vector<float> get_center_bottom(std::vector<std::vector<float>> corners_2d_homo)
{
    std::vector<float> center_bottom(2);
    for (int i = 0; i < 2; i++) {
        center_bottom[i] = (corners_2d_homo[i][2] + corners_2d_homo[i][3] + corners_2d_homo[i][7] + corners_2d_homo[i][6]) / 4;
    }
    return center_bottom;
}
std::string name_string_same_length(std::string ori_name, int length)
{
    int ori_name_len = strlen(ori_name.c_str());
    std::string new_name(length, '0');
    for (int i = 0; i < ori_name_len; i++)
    {
        new_name[length - (i + 1)] = ori_name[ori_name_len - (i + 1)];
    }
    return new_name;
}

TBevResult bev_proc(TensorConvert *parms, cv::Mat intrin, cv::Mat rot)
{
    std::vector<std::vector<float>> rot_(3, std::vector<float>(3));
    std::vector<std::vector<float>> rot_inv(3, std::vector<float>(3));
    Mat_to_vec(rot, rot_);
    matrix_inverse(rot_, 3, rot_inv);
    std::vector<std::vector<float>> intrin_(3, std::vector<float>(3));
    Mat_to_vec(intrin, intrin_);
    std::vector<std::vector<float>> corners_2d_homo_head4(4, std::vector<float>(3));
    std::vector<std::vector<float>> corners_2d_homo_tail4(4, std::vector<float>(3));
    TBevResult result;
    TBevObj obj;
    
    for (int ll = 0; ll < parms->centers.size(); ll++)
    {
        std::vector<float> center = parms->centers[ll];
        std::vector<float> quat(4);
        std::vector<float> r;
        std::vector<std::vector<float>> rotz(3, std::vector<float>(3));
        std::vector<std::vector<float>> rotz_new(3, std::vector<float>(3));
        quat[0] = parms->quats[ll][1];
        quat[1] = parms->quats[ll][2];
        quat[2] = parms->quats[ll][3];
        quat[3] = parms->quats[ll][0];
       
        quat_to_rotMatrix_scipy(quat, rotz);
        
        std::vector<float> whl = parms->whls[ll];        
       
        // �����Գ�����ϵ�İ˸�����
        float w, h, l;
        w = whl[0];
        l = whl[1];
        h = whl[2];
        std::vector<float> x_corners = { 1,  1, -1, -1};
        std::vector<float> y_corners = { 1, -1, -1,  1};
        std::vector<float> z_corners = { 1,  1,  1,  1};
        for (int i = 0; i < 4; i++) {
            x_corners[i] *= l / 2;
            y_corners[i] *= w / 2;
            z_corners[i] *= h / 2;
        }

        std::vector<std::vector<float>> corners;
        std::vector<std::vector<float>> corners_new(3, std::vector<float>(4));
        std::vector<cv::Point3d> corners_new_new(4);
        corners.push_back(x_corners);
        corners.push_back(y_corners);
        corners.push_back(z_corners);
        matrix_mul(rotz, 3, 3, corners, 4, 3, corners_new);   // 8���������ת��

       for (int i = 0; i < center.size(); i++) {
            for (int j = 0; j < corners[i].size(); j++) {
                corners_new[i][j] += center[i];                   // 8����ƽ�����������ϵ
            }
        }
        
        for (int i = 0; i < 4; i++) {
            obj.point[i].x = corners_new[0][i];
            obj.point[i].y = corners_new[1][i];            
        }

        obj.cls = parms->labels[ll];
        obj.sorce = parms->scores[ll];
        result.bev.push_back(obj);

/*
 quat = [0, 0, math.sin(rot[ll] / 2), math.cos(rot[ll] / 2)]
        r = R.from_quat(quat)
        rotz = r.as_matrix()
        w, l, h = np.array(sizes[ll])
        x_corners = l / 2 * np.array([1, 1, -1, -1 ])
        y_corners = w / 2 * np.array([1, -1, -1, 1 ])
        z_corners = h / 2 * np.array([1, 1,1, 1, ])
        corners = np.vstack((x_corners, y_corners, z_corners))  # 生成8个点
        corners = np.dot(rotz, corners)  # 8个点加入旋转角
        x, y, z = np.array(centers[ll])  

  corners1[0, :] = (corners[0, :] + x )
                corners1[1, :] = ((corners[1, :] + y))
                corners1[2, :] = corners[2, :] + z
                x1 = corners1[0,:].astype(np.float32)
                y1 = corners1[1,:].astype(np.float32)

                for iq in range(4):
                    file.write(struct.pack('<f', x1[iq]))
                    file.write(struct.pack('<f', y1[iq]))              
        */
        
    }    

    return result;
}

TImgResult projection_test(cv::Mat frame, TensorConvert *parms, cv::Mat intrin, cv::Mat tran, cv::Mat rot, int view_id, int imgs_id, cv::Mat& frame_imshow, cv::Mat xuan, cv::Mat jibian, double xi_)
{
    std::vector<std::vector<float>> rot_(3, std::vector<float>(3));
    std::vector<std::vector<float>> rot_inv(3, std::vector<float>(3));
    Mat_to_vec(rot, rot_);
    matrix_inverse(rot_, 3, rot_inv);
    std::vector<std::vector<float>> intrin_(3, std::vector<float>(3));
    Mat_to_vec(intrin, intrin_);
    std::vector<std::vector<float>> corners_2d_homo_head4(4, std::vector<float>(3));
    std::vector<std::vector<float>> corners_2d_homo_tail4(4, std::vector<float>(3));
    TImgResult result;
    TImgObj obj;
    
    for (int ll = 0; ll < parms->centers.size(); ll++)
    {
        std::vector<float> center = parms->centers[ll];
        std::vector<float> quat(4);
        std::vector<float> r;
        std::vector<std::vector<float>> rotz(3, std::vector<float>(3));
        std::vector<std::vector<float>> rotz_new(3, std::vector<float>(3));
        quat[0] = parms->quats[ll][1];
        quat[1] = parms->quats[ll][2];
        quat[2] = parms->quats[ll][3];
        quat[3] = parms->quats[ll][0];
       
        quat_to_rotMatrix_scipy(quat, rotz);
        
        std::vector<float> whl = parms->whls[ll];
        
        // �����Գ�����ϵ�İ˸�����
        float w, h, l;
        w = whl[0];
        l = whl[1];
        h = whl[2];
        std::vector<float> x_corners = { 1,  1,  1,  1, -1, -1, -1, -1 };
        std::vector<float> y_corners = { 1, -1, -1,  1,  1, -1, -1,  1 };
        std::vector<float> z_corners = { 1,  1, -1, -1,  1,  1, -1, -1 };
        for (int i = 0; i < 8; i++) {
            x_corners[i] *= l / 2;
            y_corners[i] *= w / 2;
            z_corners[i] *= h / 2;
        }
        std::vector<std::vector<float>> corners;
        std::vector<std::vector<float>> corners_new(3, std::vector<float>(8));
        std::vector<cv::Point3d> corners_new_new(8);
        corners.push_back(x_corners);
        corners.push_back(y_corners);
        corners.push_back(z_corners);                             // ����8����
        matrix_mul(rotz, 3, 3, corners, 8, 3, corners_new);   // 8���������ת��
        
        for (int i = 0; i < center.size(); i++) {
            for (int j = 0; j < corners[i].size(); j++) {
                corners_new[i][j] += center[i];                   // 8����ƽ�����������ϵ
            }
        }
        // std::cout << "kkkkkkkkkkkkkkkkkkkkkkkkkkkasssss" << std::endl;
        for (int i = 0; i < 8; i++) {
            corners_new_new[i].x = corners_new[0][i];
            corners_new_new[i].y = corners_new[1][i];
            corners_new_new[i].z = corners_new[2][i];
        }
        // ����ͼ������ϵ�İ˸���������ĵ�����
        // ================================
        std::vector<cv::Point2d> corners_2d_omnidir;
        // std::cout << "asssss" << corners_2d_omnidir << std::endl;
        cv::omnidir::projectPoints(corners_new_new, corners_2d_omnidir, xuan, tran, intrin, xi_, jibian);

        
        //cv::omnidir(corners_new_new, corners_2d_omnidir, xuan, tran, intrin, xi_, jibian);
        // ================================        
        for (int idd = 0; idd < 8; idd++)
        {
            obj.point[idd].x = corners_2d_omnidir[idd].x;
            obj.point[idd].y = corners_2d_omnidir[idd].y;             
        }
        obj.cls = parms->labels[ll];
        obj.sorce = parms->scores[ll];
        result.img.push_back(obj);

        #if SHOW_IN_PICTURE
        //cv::omnidir(corners_new_new, corners_2d_omnidir, xuan, tran, intrin, xi_, jibian);
        // ================================                
        std::vector<std::vector<float>> corners_2d_homo(2, std::vector<float>(8));
        for (int idd = 0; idd < 8; idd++)
        {
            corners_2d_homo[0][idd] = corners_2d_omnidir[idd].x;
            corners_2d_homo[1][idd] = corners_2d_omnidir[idd].y;            
        }        
        int nn = 0;
        for (int g = 0; g < 8; g++)
        {
            if (corners_2d_homo[0][g] > 0 && corners_2d_homo[0][g] < 1920 && corners_2d_homo[1][g] > 0 && corners_2d_homo[1][g] < 1080)
            {
                nn += 1;
                if (nn >= 2)
                {
                    for (int i = 0; i < 4; i++) {
                        cv::line(frame, cv::Point((int)(corners_2d_homo[0][i]), (int)(corners_2d_homo[1][i])), cv::Point((int)(corners_2d_homo[0][i + 4]), (int)(corners_2d_homo[1][i + 4])), cv::Scalar(0, 0, 255), 2);
                    }
                    // get head and tail corners
                    for (int id = 0; id < 4; id++) {
                        corners_2d_homo_head4[id][0] = corners_2d_homo[0][id];
                        corners_2d_homo_head4[id][1] = corners_2d_homo[1][id];
                        corners_2d_homo_tail4[id][0] = corners_2d_homo[0][id + 4];
                        corners_2d_homo_tail4[id][1] = corners_2d_homo[1][id + 4];
                    }
                    draw_cv(frame, corners_2d_homo_head4);
                    draw_cv(frame, corners_2d_homo_tail4);
                    std::vector<float> center_bottom_forward = get_center_bottom_forward(corners_2d_homo);
                    std::vector<float> center_bottom = get_center_bottom(corners_2d_homo);
                    cv::line(frame, cv::Point((int)(center_bottom[0]), (int)(center_bottom[1])),
                        cv::Point((int)(center_bottom_forward[0]), (int)(center_bottom_forward[1])), cv::Scalar(0, 0, 255), 2);
                }
            }
        }
        #endif        
    }
    
    #if SHOW_IN_PICTURE
    cv::resize(frame, frame, cv::Size(800, 450));

    char save_path[128];
    snprintf(save_path, sizeof(save_path), "%d-out.jpg", view_id);
    printf("save_path:%s\n",save_path);
    cv::imwrite(save_path, frame);
    #endif

    return result;
}

TImgResult projection(TensorConvert *parms, cv::Mat intrin, cv::Mat tran, cv::Mat rot, int view_id, int imgs_id, cv::Mat& frame_imshow, cv::Mat xuan, cv::Mat jibian, double xi_)
{
    std::vector<std::vector<float>> rot_(3, std::vector<float>(3));
    std::vector<std::vector<float>> rot_inv(3, std::vector<float>(3));
    Mat_to_vec(rot, rot_);
    matrix_inverse(rot_, 3, rot_inv);
    std::vector<std::vector<float>> intrin_(3, std::vector<float>(3));
    Mat_to_vec(intrin, intrin_);
    std::vector<std::vector<float>> corners_2d_homo_head4(4, std::vector<float>(3));
    std::vector<std::vector<float>> corners_2d_homo_tail4(4, std::vector<float>(3));
    TImgResult result;
    TImgObj obj;
    
    for (int ll = 0; ll < parms->centers.size(); ll++)
    {
        std::vector<float> center = parms->centers[ll];
        std::vector<float> quat(4);
        std::vector<float> r;
        std::vector<std::vector<float>> rotz(3, std::vector<float>(3));
        std::vector<std::vector<float>> rotz_new(3, std::vector<float>(3));
        quat[0] = parms->quats[ll][1];
        quat[1] = parms->quats[ll][2];
        quat[2] = parms->quats[ll][3];
        quat[3] = parms->quats[ll][0];
       
        quat_to_rotMatrix_scipy(quat, rotz);
        
        std::vector<float> whl = parms->whls[ll];
        
        // �����Գ�����ϵ�İ˸�����
        float w, h, l;
        w = whl[0];
        l = whl[1];
        h = whl[2];
        std::vector<float> x_corners = { 1,  1,  1,  1, -1, -1, -1, -1 };
        std::vector<float> y_corners = { 1, -1, -1,  1,  1, -1, -1,  1 };
        std::vector<float> z_corners = { 1,  1, -1, -1,  1,  1, -1, -1 };
        for (int i = 0; i < 8; i++) {
            x_corners[i] *= l / 2;
            y_corners[i] *= w / 2;
            z_corners[i] *= h / 2;
        }
        std::vector<std::vector<float>> corners;
        std::vector<std::vector<float>> corners_new(3, std::vector<float>(8));
        std::vector<cv::Point3d> corners_new_new(8);
        corners.push_back(x_corners);
        corners.push_back(y_corners);
        corners.push_back(z_corners);                             // ����8����
        matrix_mul(rotz, 3, 3, corners, 8, 3, corners_new);   // 8���������ת��
        
        for (int i = 0; i < center.size(); i++) {
            for (int j = 0; j < corners[i].size(); j++) {
                corners_new[i][j] += center[i];                   // 8����ƽ�����������ϵ
            }
        }
        // std::cout << "kkkkkkkkkkkkkkkkkkkkkkkkkkkasssss" << std::endl;
        for (int i = 0; i < 8; i++) {
            corners_new_new[i].x = corners_new[0][i];
            corners_new_new[i].y = corners_new[1][i];
            corners_new_new[i].z = corners_new[2][i];
        }
        // ����ͼ������ϵ�İ˸���������ĵ�����
        // ================================
        std::vector<cv::Point2d> corners_2d_omnidir;
        // std::cout << "asssss" << corners_2d_omnidir << std::endl;
        cv::omnidir::projectPoints(corners_new_new, corners_2d_omnidir, xuan, tran, intrin, xi_, jibian);

        
        //cv::omnidir(corners_new_new, corners_2d_omnidir, xuan, tran, intrin, xi_, jibian);
        // ================================        
        for (int idd = 0; idd < 8; idd++)
        {
            obj.point[idd].x = corners_2d_omnidir[idd].x;
            obj.point[idd].y = corners_2d_omnidir[idd].y;             
        }
        obj.cls = parms->labels[ll];
        obj.sorce = parms->scores[ll];
        result.img.push_back(obj);       
    }

    return result;
}

