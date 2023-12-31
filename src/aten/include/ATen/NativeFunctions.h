#pragma once

#include "ATen/ATen.h"
#include <tuple>
#include <vector>

namespace at {
namespace native {

Tensor adaptive_avg_pool1d(const Tensor & self, IntList output_size);
std::tuple<Tensor,Tensor> adaptive_max_pool1d(const Tensor & self, IntList output_size);
bool allclose(const Tensor & self, const Tensor & other, double rtol=1e-05, double atol=1e-08);
Tensor batch_norm(const Tensor & input, const Tensor & weight, const Tensor & bias, const Tensor & running_mean, const Tensor & running_var, bool training, double momentum, double eps, bool cudnn_enabled);
Tensor & bernoulli_(Tensor & self, const Tensor & p, Generator * generator=nullptr);
Tensor & bernoulli_(Tensor & self, double p, Generator * generator=nullptr);
std::vector<Tensor> chunk(const Tensor & self, int64_t chunks, int64_t dim=0);
bool cudnn_is_acceptable(const Tensor & self);
Tensor convolution(const Tensor & input, const Tensor & weight, const Tensor & bias, IntList stride, IntList padding, IntList dilation, bool transposed, IntList output_padding, int64_t groups);
Tensor _convolution(const Tensor & input, const Tensor & weight, const Tensor & bias, IntList stride, IntList padding, IntList dilation, bool transposed, IntList output_padding, int64_t groups, bool benchmark, bool deterministic, bool cudnn_enabled);
Tensor _convolution_nogroup(const Tensor & input, const Tensor & weight, const Tensor & bias, IntList stride, IntList padding, IntList dilation, bool transposed, IntList output_padding);
std::tuple<Tensor,Tensor,Tensor> _convolution_double_backward(const Tensor & ggI, const Tensor & ggW, const Tensor & ggb, const Tensor & gO, const Tensor & weight, const Tensor & self, IntList stride, IntList padding, IntList dilation, bool transposed, IntList output_padding, int64_t groups, bool benchmark, bool deterministic, bool cudnn_enabled, std::array<bool,3> output_mask);
Tensor conv1d(const Tensor & input, const Tensor & weight, const Tensor & bias={}, IntList stride=1, IntList padding=0, IntList dilation=1, int64_t groups=1);
Tensor conv2d(const Tensor & input, const Tensor & weight, const Tensor & bias={}, IntList stride=1, IntList padding=0, IntList dilation=1, int64_t groups=1);
Tensor conv3d(const Tensor & input, const Tensor & weight, const Tensor & bias={}, IntList stride=1, IntList padding=0, IntList dilation=1, int64_t groups=1);
Tensor conv_tbc(const Tensor & self, const Tensor & weight, const Tensor & bias, int64_t pad);
std::tuple<Tensor,Tensor,Tensor> conv_tbc_backward(const Tensor & self, const Tensor & input, const Tensor & weight, const Tensor & bias, int64_t pad);
Tensor conv_transpose1d(const Tensor & input, const Tensor & weight, const Tensor & bias={}, IntList stride=1, IntList padding=0, IntList output_padding=0, int64_t groups=1, IntList dilation=1);
Tensor conv_transpose2d(const Tensor & input, const Tensor & weight, const Tensor & bias={}, IntList stride=1, IntList padding=0, IntList output_padding=0, int64_t groups=1, IntList dilation=1);
Tensor conv_transpose3d(const Tensor & input, const Tensor & weight, const Tensor & bias={}, IntList stride=1, IntList padding=0, IntList output_padding=0, int64_t groups=1, IntList dilation=1);
Tensor cudnn_affine_grid_generator_forward(const Tensor & theta, int64_t N, int64_t C, int64_t H, int64_t W);
Tensor cudnn_affine_grid_generator_backward(const Tensor & grad, int64_t N, int64_t C, int64_t H, int64_t W);
std::tuple<Tensor,Tensor,Tensor> cudnn_batch_norm(const Tensor & input, const Tensor & weight, const Tensor & bias, const Tensor & running_mean, const Tensor & running_var, bool training, double exponential_average_factor, double epsilon);
std::tuple<Tensor,Tensor,Tensor> cudnn_batch_norm_backward(const Tensor & input, const Tensor & grad_output, const Tensor & weight, const Tensor & running_mean, const Tensor & running_var, const Tensor & save_mean, const Tensor & save_var, double epsilon);
Tensor cudnn_convolution(const Tensor & self, const Tensor & weight, const Tensor & bias, IntList padding, IntList stride, IntList dilation, int64_t groups, bool benchmark, bool deterministic);
Tensor cudnn_convolution_backward_input(IntList self_size, const Tensor & grad_output, const Tensor & weight, IntList padding, IntList stride, IntList dilation, int64_t groups, bool benchmark, bool deterministic);
std::tuple<Tensor,Tensor,Tensor> cudnn_convolution_backward(const Tensor & self, const Tensor & grad_output, const Tensor & weight, IntList padding, IntList stride, IntList dilation, int64_t groups, bool benchmark, bool deterministic, std::array<bool,3> output_mask);
Tensor cudnn_convolution_backward_bias(const Tensor & grad_output);
Tensor cudnn_convolution_backward_weight(IntList weight_size, const Tensor & grad_output, const Tensor & self, IntList padding, IntList stride, IntList dilation, int64_t groups, bool benchmark, bool deterministic);
Tensor cudnn_convolution_transpose(const Tensor & self, const Tensor & weight, const Tensor & bias, IntList padding, IntList output_padding, IntList stride, IntList dilation, int64_t groups, bool benchmark, bool deterministic);
std::tuple<Tensor,Tensor,Tensor> cudnn_convolution_transpose_backward(const Tensor & self, const Tensor & grad_output, const Tensor & weight, IntList padding, IntList output_padding, IntList stride, IntList dilation, int64_t groups, bool benchmark, bool deterministic, std::array<bool,3> output_mask);
Tensor cudnn_convolution_backward_bias(const Tensor & grad_output);
Tensor cudnn_convolution_transpose_backward_input(const Tensor & grad_output, const Tensor & weight, IntList padding, IntList stride, IntList dilation, int64_t groups, bool benchmark, bool deterministic);
Tensor cudnn_convolution_transpose_backward_weight(IntList weight_size, const Tensor & grad_output, const Tensor & self, IntList padding, IntList stride, IntList dilation, int64_t groups, bool benchmark, bool deterministic);
Tensor cudnn_grid_sampler_forward(const Tensor & self, const Tensor & grid);
std::tuple<Tensor,Tensor> cudnn_grid_sampler_backward(const Tensor & self, const Tensor & grid, const Tensor & grad_output);
Tensor det(const Tensor & self);
std::tuple<Tensor,Tensor,Tensor,Tensor> _det_with_svd(const Tensor & self);
Tensor embedding(const Tensor & weight, const Tensor & indices, int64_t padding_idx=-1, bool scale_grad_by_freq=false, bool sparse=false);
Tensor embedding_backward(const Tensor & grad, const Tensor & indices, int64_t num_weights, int64_t padding_idx, bool scale_grad_by_freq, bool sparse);
Tensor embedding_backward_cpu(const Tensor & grad, const Tensor & indices, int64_t num_weights, int64_t padding_idx, bool scale_grad_by_freq);
Tensor embedding_backward_cuda(const Tensor & grad, const Tensor & indices, int64_t num_weights, int64_t padding_idx, bool scale_grad_by_freq);
Tensor & embedding_renorm_cpu_(Tensor & self, const Tensor & indices, double max_norm, double norm_type);
Tensor & embedding_renorm_cuda_(Tensor & self, const Tensor & indices, double max_norm, double norm_type);
Tensor embedding_sparse_backward(const Tensor & grad, const Tensor & indices, int64_t num_weights, int64_t padding_idx, bool scale_grad_by_freq);
Tensor empty_like(const Tensor & self);
Tensor expand(const Tensor & self, IntList size);
Tensor expand_as(const Tensor & self, const Tensor & other);
Tensor index(const Tensor & self, TensorList indices);
Tensor & index_put_(Tensor & self, TensorList indices, const Tensor & values);
bool is_cuda(const Tensor & self);
bool is_distributed(const Tensor & self);
bool is_floating_point(const Tensor & self);
bool is_nonzero(const Tensor & self);
bool is_same_size(const Tensor & self, const Tensor & other);
bool is_signed(const Tensor & self);
bool is_sparse(const Tensor & self);
Tensor matmul(const Tensor & self, const Tensor & other);
std::tuple<Tensor,Tensor> max_pool1d(const Tensor & self, IntList kernel_size, IntList stride={}, IntList padding=0, IntList dilation=1, bool ceil_mode=false);
Tensor narrow(const Tensor & self, int64_t dim, int64_t start, int64_t length);
Tensor nnpack_spatial_convolution(const Tensor & input, const Tensor & weight, const Tensor & bias, int64_t kW, int64_t kH, int64_t padW, int64_t padH);
std::tuple<Tensor,Tensor,Tensor> nnpack_spatial_convolution_backward(const Tensor & input, const Tensor & grad_output, const Tensor & weight, int64_t kW, int64_t kH, int64_t padW, int64_t padH, std::array<bool,3> output_mask);
Tensor nnpack_spatial_convolution_backward_input(const Tensor & input, const Tensor & grad_output, const Tensor & weight, int64_t kW, int64_t kH, int64_t padW, int64_t padH);
Tensor nnpack_spatial_convolution_backward_weight(const Tensor & input, IntList weight_size, const Tensor & grad_output, int64_t kW, int64_t kH, int64_t padW, int64_t padH);
Tensor permute(const Tensor & self, IntList dims);
Tensor pin_memory(const Tensor & self);
Tensor randn_like(const Tensor & self);
std::tuple<Tensor,Tensor> RoiPooling2d_forward_cpu(const Tensor & input, const Tensor & rois, int64_t pooledHeight, int64_t pooledWidth, double spatialScale);
std::tuple<Tensor,Tensor> RoiPooling2d_forward_cuda(const Tensor & input, const Tensor & rois, int64_t pooledHeight, int64_t pooledWidth, double spatialScale);
Tensor RoiPooling2d_backward_cpu(const Tensor & input, const Tensor & rois, int64_t pooledHeight, int64_t pooledWidth, double spatialScale, const Tensor & gradOutput, const Tensor & argmaxes);
Tensor RoiPooling2d_backward_cuda(const Tensor & input, const Tensor & rois, int64_t pooledHeight, int64_t pooledWidth, double spatialScale, const Tensor & gradOutput, const Tensor & argmaxes);
Tensor rrelu(const Tensor & self, Scalar lower=0.125, Scalar upper=0.333333333333, bool training=false, Generator * generator=nullptr);
Tensor & rrelu_(Tensor & self, Scalar lower=0.125, Scalar upper=0.333333333333, bool training=false, Generator * generator=nullptr);
Tensor select(const Tensor & self, int64_t dim, int64_t index);
Tensor selu(const Tensor & self);
Tensor & selu_(Tensor & self);
int64_t size(const Tensor & self, int64_t dim);
Tensor slice(const Tensor & self, int64_t dim=0, int64_t start=0, int64_t end=9223372036854775807, int64_t step=1);
std::vector<Tensor> split(const Tensor & self, int64_t split_size, int64_t dim=0);
Tensor squeeze(const Tensor & self);
Tensor squeeze(const Tensor & self, int64_t dim);
Tensor & squeeze_(Tensor & self);
Tensor & squeeze_(Tensor & self, int64_t dim);
Tensor stack(TensorList tensors, int64_t dim=0);
Tensor stft(const Tensor & self, int64_t frame_length, int64_t hop, int64_t fft_size, bool return_onesided=true, const Tensor & window={}, int64_t pad_end=0);
int64_t stride(const Tensor & self, int64_t dim);
Tensor & transpose_(Tensor & self, int64_t dim0, int64_t dim1);
Tensor & t_(Tensor & self);
Tensor type_as(const Tensor & self, const Tensor & other);
Tensor unsqueeze(const Tensor & self, int64_t dim);
Tensor & unsqueeze_(Tensor & self, int64_t dim);
Tensor view_as(const Tensor & self, const Tensor & other);
Tensor where(const Tensor & condition, const Tensor & self, const Tensor & other);
Tensor _s_where_cpu(const Tensor & condition, const Tensor & self, const Tensor & other);
Tensor _s_where_cuda(const Tensor & condition, const Tensor & self, const Tensor & other);
Tensor _standard_gamma_grad_cpu(const Tensor & self, const Tensor & output);
Tensor _standard_gamma_grad_cuda(const Tensor & self, const Tensor & output);
Tensor _s_poisson_cpu(const Tensor & self, Generator * generator=nullptr);
Tensor _s_poisson_cuda(const Tensor & self, Generator * generator=nullptr);

}
}
