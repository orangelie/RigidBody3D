#include "Utils.h"

namespace Utils
{
    namespace MathTool
    {
        void Scalar(DirectX::XMFLOAT4& V, float scalar)
        {
            V.x *= scalar;
            V.y *= scalar;
            V.z *= scalar;
        }

        void AddScaledVector(DirectX::XMFLOAT4& V1, const DirectX::XMFLOAT4& V2, float scale)
        {
            V1.x += V2.x * scale;
            V1.y += V2.y * scale;
            V1.z += V2.z * scale;
        }

        void AddScaledQuaternion(DirectX::XMFLOAT4& Q, const DirectX::XMFLOAT4& V, float scale)
        {
            DirectX::XMFLOAT4 nQ = {};
            nQ.x = V.x * scale;
            nQ.y = V.y * scale;
            nQ.z = V.z * scale;
            nQ.w = 0.0f;

            DirectX::XMVECTOR QVec = DirectX::XMLoadFloat4(&Q);
            DirectX::XMVECTOR nQVec = DirectX::XMLoadFloat4(&nQ);
            DirectX::XMStoreFloat4(&nQ, DirectX::XMQuaternionMultiply(nQVec, QVec));

            Q.x += nQ.x * 0.5f;
            Q.y += nQ.y * 0.5f;
            Q.z += nQ.z * 0.5f;
            Q.w += nQ.w * 0.5f;
        }

        void Transform(DirectX::XMFLOAT4& Out, const DirectX::XMFLOAT3X4& M, const DirectX::XMFLOAT4& V)
        {
            Out.x = M.m[0][0] * V.x + M.m[0][1] * V.y + M.m[0][2] * V.z;
            Out.y = M.m[1][0] * V.x + M.m[1][1] * V.y + M.m[1][2] * V.z;
            Out.z = M.m[2][0] * V.x + M.m[2][1] * V.y + M.m[2][2] * V.z;
        }

        void TransformTranspose(DirectX::XMFLOAT4& Out, const DirectX::XMFLOAT3X4& M, const DirectX::XMFLOAT4& V)
        {
            Out.x = M.m[0][0] * V.x + M.m[1][0] * V.y + M.m[2][0] * V.z;
            Out.y = M.m[0][1] * V.x + M.m[1][1] * V.y + M.m[2][1] * V.z;
            Out.z = M.m[0][2] * V.x + M.m[1][2] * V.y + M.m[2][2] * V.z;
        }

        DirectX::XMFLOAT3X4 SetSkewSymmetric(const DirectX::XMFLOAT4& V)
        {
            DirectX::XMFLOAT3X4 M = {};
            
            M.m[0][0] = 0.0f;   M.m[0][1] = -V.z;   M.m[0][2] = V.y;   M.m[0][3] = 0.0f;
            M.m[1][0] = V.z;    M.m[1][1] = 0.0f;   M.m[1][2] = -V.x;  M.m[1][3] = 0.0f;
            M.m[2][0] = -V.y;   M.m[2][1] = V.x;    M.m[2][2] = 0.0f;  M.m[2][3] = 0.0f;

            return M;
        }

        void InertiaTensorCoeffs(DirectX::XMFLOAT3X4& InertiaTensor, float ix, float iy, float iz, float ixy, float ixz, float iyz)
        {
            InertiaTensor.m[0][0] = ix;     InertiaTensor.m[0][1] = -ixy;   InertiaTensor.m[0][2] = -ixz;   InertiaTensor.m[0][3] = 0;
            InertiaTensor.m[1][0] = -ixy;   InertiaTensor.m[1][1] = iy;     InertiaTensor.m[1][2] = -iyz;   InertiaTensor.m[1][3] = 0;
            InertiaTensor.m[2][0] = -ixz;   InertiaTensor.m[2][1] = -iyz;   InertiaTensor.m[2][2] = iz;     InertiaTensor.m[2][3] = 0;
        }

        void BlockInertiaTensor(DirectX::XMFLOAT3X4& InertiaTensor, const DirectX::XMFLOAT4& halfSizes, const float& mass)
        {
            DirectX::XMFLOAT4 squared = { halfSizes.x * halfSizes.x, halfSizes.y * halfSizes.y, halfSizes.z * halfSizes.z, halfSizes.w };

            const float coeff = mass * 0.3f;
            InertiaTensorCoeffs(InertiaTensor, coeff * (squared.y + squared.z),
                coeff * (squared.x + squared.z), coeff * (squared.x + squared.y));
        }

        void SetComponents(DirectX::XMFLOAT3X4& Comp, const DirectX::XMFLOAT4& V1, const DirectX::XMFLOAT4& V2, const DirectX::XMFLOAT4& V3)
        {
            Comp.m[0][0] = V1.x;    Comp.m[0][1] = V2.x;    Comp.m[0][2] = V3.x;    Comp.m[0][3] = 0.0f;
            Comp.m[1][0] = V1.y;    Comp.m[1][1] = V2.y;    Comp.m[1][2] = V3.y;    Comp.m[1][3] = 0.0f;
            Comp.m[2][0] = V1.z;    Comp.m[2][1] = V2.z;    Comp.m[2][2] = V3.z;    Comp.m[2][3] = 0.0f;
            Comp.m[3][0] = 0.0f;    Comp.m[3][1] = 0.0f;    Comp.m[3][2] = 0.0f;    Comp.m[3][3] = 1.0f;
        }

        DirectX::XMFLOAT3X4 SetInverse(const DirectX::XMFLOAT3X4& M)
        {
            DirectX::XMFLOAT3X4 data = Utils::MatrixIdentity3();

            float t4 = M.m[0][0] * M.m[1][1];
            float t6 = M.m[0][0] * M.m[1][2];
            float t8 = M.m[0][1] * M.m[1][0];
            float t10 = M.m[0][2] * M.m[1][0];
            float t12 = M.m[0][1] * M.m[2][0];
            float t14 = M.m[0][2] * M.m[2][0];

            // Calculate the determinant
            float t16 = (t4 * M.m[2][2] - t6 * M.m[2][1] - t8 * M.m[2][2] +
                t10 * M.m[2][1] + t12 * M.m[1][2] - t14 * M.m[1][1]);

            // Make sure the determinant is non-zero.
            if (t16 == (float)0.0f) return Utils::MatrixIdentity3();
            float t17 = 1.0f / t16;

            data.m[0][0] = (M.m[1][1] * M.m[2][2] - M.m[1][2] * M.m[2][1]) * t17;
            data.m[0][1] = -(M.m[0][1] * M.m[2][2] - M.m[0][2] * M.m[2][1]) * t17;
            data.m[0][2] = (M.m[0][1] * M.m[1][2] - M.m[0][2] * M.m[1][1]) * t17;
            data.m[1][0] = -(M.m[1][0] * M.m[2][2] - M.m[1][2] * M.m[2][0]) * t17;
            data.m[1][1] = (M.m[0][0] * M.m[2][2] - t14) * t17;
            data.m[1][2] = -(t6 - t10) * t17;
            data.m[2][0] = (M.m[1][0] * M.m[2][1] - M.m[1][1] * M.m[2][0]) * t17;
            data.m[2][1] = -(M.m[0][0] * M.m[2][1] - t12) * t17;
            data.m[2][2] = (t4 - t8) * t17;

            data.m[3][3] = 1.0f;

            return data;
        }

        void PrintVector(const DirectX::XMFLOAT4& V)
        {
            std::string str =
                std::to_string(V.x) + ", " + std::to_string(V.y) + ", " + std::to_string(V.z) + ", " + std::to_string(V.w);

            MessageBoxA(0, str.c_str(), "", MB_OK);
        }

        void PrintMatrix(const DirectX::XMFLOAT3X4& M)
        {
            std::string str =
                std::to_string(M.m[0][0]) + ", " + std::to_string(M.m[0][1]) + ", " + std::to_string(M.m[0][2]) + ", " + std::to_string(M.m[0][3]) + "\n" +
                std::to_string(M.m[1][0]) + ", " + std::to_string(M.m[1][1]) + ", " + std::to_string(M.m[1][2]) + ", " + std::to_string(M.m[1][3]) + "\n" +
                std::to_string(M.m[2][0]) + ", " + std::to_string(M.m[2][1]) + ", " + std::to_string(M.m[2][2]) + ", " + std::to_string(M.m[2][3]);

            MessageBoxA(0, str.c_str(), "", MB_OK);
        }
    }

    DirectX::XMFLOAT4X4 MatrixIdentity()
    {
        static DirectX::XMFLOAT4X4 identity = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };

        return identity;
    }

    DirectX::XMFLOAT3X4 MatrixIdentity3()
    {
        static DirectX::XMFLOAT3X4 identity = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f
        };

        return identity;
    }

    UINT ConstantBufferSize(UINT size)
    {
        return (size + 255) & ~255;
    }

    Microsoft::WRL::ComPtr<ID3D12Resource> CreateDefaultResource(
        ID3D12Device* device,
        ID3D12GraphicsCommandList* cmdList,
        const void* data,
        size_t size,
        Microsoft::WRL::ComPtr<ID3D12Resource>& uploadHeap)
    {
        Microsoft::WRL::ComPtr<ID3D12Resource> defaultHeap = nullptr;

        HR(device->CreateCommittedResource(&unmove(CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT)), D3D12_HEAP_FLAG_NONE,
            &unmove(CD3DX12_RESOURCE_DESC::Buffer(size)), D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(defaultHeap.GetAddressOf())));
        HR(device->CreateCommittedResource(&unmove(CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD)), D3D12_HEAP_FLAG_NONE,
            &unmove(CD3DX12_RESOURCE_DESC::Buffer(size)), D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(uploadHeap.GetAddressOf())));

        D3D12_SUBRESOURCE_DATA subResourceData = {};
        subResourceData.pData = data;
        subResourceData.RowPitch = (LONG_PTR)size;
        subResourceData.SlicePitch = subResourceData.RowPitch;

        cmdList->ResourceBarrier(1, &unmove(CD3DX12_RESOURCE_BARRIER::Transition(defaultHeap.Get(),
            D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST)));
        UpdateSubresources<1>(cmdList, defaultHeap.Get(), uploadHeap.Get(), 0, 0, 1, &subResourceData);
        cmdList->ResourceBarrier(1, &unmove(CD3DX12_RESOURCE_BARRIER::Transition(defaultHeap.Get(),
            D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_GENERIC_READ)));

        return defaultHeap;
    }

    Microsoft::WRL::ComPtr<ID3DBlob> CompileShader(
        const std::wstring& filename,
        const D3D_SHADER_MACRO* macro,
        const std::string& entryPoint,
        const std::string& target)
    {
        UINT compileFlags = D3DCOMPILE_ENABLE_STRICTNESS;

#if defined(DEBUG) || defined(_DEBUG)
        compileFlags |= D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

        Microsoft::WRL::ComPtr<ID3DBlob> ppCode = nullptr, ppErrorMsgs = nullptr;

        HR(D3DCompileFromFile(filename.c_str(), macro, D3D_COMPILE_STANDARD_FILE_INCLUDE,
            entryPoint.c_str(), target.c_str(), compileFlags, 0, ppCode.GetAddressOf(), ppErrorMsgs.GetAddressOf()));

        if (ppErrorMsgs != nullptr)
        {
            MessageBoxA(0, (char*)ppErrorMsgs->GetBufferPointer(), "Compiler Error", MB_OK);
            return nullptr;
        }

        return ppCode;
    }
}

namespace WICConverter
{
    // get the dxgi format equivilent of a wic format
    DXGI_FORMAT GetDXGIFormatFromWICFormat(WICPixelFormatGUID& wicFormatGUID)
    {
        if (wicFormatGUID == GUID_WICPixelFormat128bppRGBAFloat) return DXGI_FORMAT_R32G32B32A32_FLOAT;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppRGBAHalf) return DXGI_FORMAT_R16G16B16A16_FLOAT;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppRGBA) return DXGI_FORMAT_R16G16B16A16_UNORM;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppRGBA) return DXGI_FORMAT_R8G8B8A8_UNORM;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppBGRA) return DXGI_FORMAT_B8G8R8A8_UNORM;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppBGR) return DXGI_FORMAT_B8G8R8X8_UNORM;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppRGBA1010102XR) return DXGI_FORMAT_R10G10B10_XR_BIAS_A2_UNORM;

        else if (wicFormatGUID == GUID_WICPixelFormat32bppRGBA1010102) return DXGI_FORMAT_R10G10B10A2_UNORM;
        else if (wicFormatGUID == GUID_WICPixelFormat16bppBGRA5551) return DXGI_FORMAT_B5G5R5A1_UNORM;
        else if (wicFormatGUID == GUID_WICPixelFormat16bppBGR565) return DXGI_FORMAT_B5G6R5_UNORM;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppGrayFloat) return DXGI_FORMAT_R32_FLOAT;
        else if (wicFormatGUID == GUID_WICPixelFormat16bppGrayHalf) return DXGI_FORMAT_R16_FLOAT;
        else if (wicFormatGUID == GUID_WICPixelFormat16bppGray) return DXGI_FORMAT_R16_UNORM;
        else if (wicFormatGUID == GUID_WICPixelFormat8bppGray) return DXGI_FORMAT_R8_UNORM;
        else if (wicFormatGUID == GUID_WICPixelFormat8bppAlpha) return DXGI_FORMAT_A8_UNORM;

        else return DXGI_FORMAT_UNKNOWN;
    }

    // get a dxgi compatible wic format from another wic format
    WICPixelFormatGUID GetConvertToWICFormat(WICPixelFormatGUID& wicFormatGUID)
    {
        if (wicFormatGUID == GUID_WICPixelFormatBlackWhite) return GUID_WICPixelFormat8bppGray;
        else if (wicFormatGUID == GUID_WICPixelFormat1bppIndexed) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat2bppIndexed) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat4bppIndexed) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat8bppIndexed) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat2bppGray) return GUID_WICPixelFormat8bppGray;
        else if (wicFormatGUID == GUID_WICPixelFormat4bppGray) return GUID_WICPixelFormat8bppGray;
        else if (wicFormatGUID == GUID_WICPixelFormat16bppGrayFixedPoint) return GUID_WICPixelFormat16bppGrayHalf;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppGrayFixedPoint) return GUID_WICPixelFormat32bppGrayFloat;
        else if (wicFormatGUID == GUID_WICPixelFormat16bppBGR555) return GUID_WICPixelFormat16bppBGRA5551;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppBGR101010) return GUID_WICPixelFormat32bppRGBA1010102;
        else if (wicFormatGUID == GUID_WICPixelFormat24bppBGR) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat24bppRGB) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppPBGRA) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppPRGBA) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat48bppRGB) return GUID_WICPixelFormat64bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat48bppBGR) return GUID_WICPixelFormat64bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppBGRA) return GUID_WICPixelFormat64bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppPRGBA) return GUID_WICPixelFormat64bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppPBGRA) return GUID_WICPixelFormat64bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat48bppRGBFixedPoint) return GUID_WICPixelFormat64bppRGBAHalf;
        else if (wicFormatGUID == GUID_WICPixelFormat48bppBGRFixedPoint) return GUID_WICPixelFormat64bppRGBAHalf;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppRGBAFixedPoint) return GUID_WICPixelFormat64bppRGBAHalf;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppBGRAFixedPoint) return GUID_WICPixelFormat64bppRGBAHalf;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppRGBFixedPoint) return GUID_WICPixelFormat64bppRGBAHalf;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppRGBHalf) return GUID_WICPixelFormat64bppRGBAHalf;
        else if (wicFormatGUID == GUID_WICPixelFormat48bppRGBHalf) return GUID_WICPixelFormat64bppRGBAHalf;
        else if (wicFormatGUID == GUID_WICPixelFormat128bppPRGBAFloat) return GUID_WICPixelFormat128bppRGBAFloat;
        else if (wicFormatGUID == GUID_WICPixelFormat128bppRGBFloat) return GUID_WICPixelFormat128bppRGBAFloat;
        else if (wicFormatGUID == GUID_WICPixelFormat128bppRGBAFixedPoint) return GUID_WICPixelFormat128bppRGBAFloat;
        else if (wicFormatGUID == GUID_WICPixelFormat128bppRGBFixedPoint) return GUID_WICPixelFormat128bppRGBAFloat;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppRGBE) return GUID_WICPixelFormat128bppRGBAFloat;
        else if (wicFormatGUID == GUID_WICPixelFormat32bppCMYK) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppCMYK) return GUID_WICPixelFormat64bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat40bppCMYKAlpha) return GUID_WICPixelFormat64bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat80bppCMYKAlpha) return GUID_WICPixelFormat64bppRGBA;

#if (_WIN32_WINNT >= _WIN32_WINNT_WIN8) || defined(_WIN7_PLATFORM_UPDATE)
        else if (wicFormatGUID == GUID_WICPixelFormat32bppRGB) return GUID_WICPixelFormat32bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppRGB) return GUID_WICPixelFormat64bppRGBA;
        else if (wicFormatGUID == GUID_WICPixelFormat64bppPRGBAHalf) return GUID_WICPixelFormat64bppRGBAHalf;
#endif

        else return GUID_WICPixelFormatDontCare;
    }

    // get the number of bits per pixel for a dxgi format
    int GetDXGIFormatBitsPerPixel(DXGI_FORMAT& dxgiFormat)
    {
        if (dxgiFormat == DXGI_FORMAT_R32G32B32A32_FLOAT) return 128;
        else if (dxgiFormat == DXGI_FORMAT_R16G16B16A16_FLOAT) return 64;
        else if (dxgiFormat == DXGI_FORMAT_R16G16B16A16_UNORM) return 64;
        else if (dxgiFormat == DXGI_FORMAT_R8G8B8A8_UNORM) return 32;
        else if (dxgiFormat == DXGI_FORMAT_B8G8R8A8_UNORM) return 32;
        else if (dxgiFormat == DXGI_FORMAT_B8G8R8X8_UNORM) return 32;
        else if (dxgiFormat == DXGI_FORMAT_R10G10B10_XR_BIAS_A2_UNORM) return 32;

        else if (dxgiFormat == DXGI_FORMAT_R10G10B10A2_UNORM) return 32;
        else if (dxgiFormat == DXGI_FORMAT_B5G5R5A1_UNORM) return 16;
        else if (dxgiFormat == DXGI_FORMAT_B5G6R5_UNORM) return 16;
        else if (dxgiFormat == DXGI_FORMAT_R32_FLOAT) return 32;
        else if (dxgiFormat == DXGI_FORMAT_R16_FLOAT) return 16;
        else if (dxgiFormat == DXGI_FORMAT_R16_UNORM) return 16;
        else if (dxgiFormat == DXGI_FORMAT_R8_UNORM) return 8;
        else if (dxgiFormat == DXGI_FORMAT_A8_UNORM) return 8;

        return 8;
    }

    // load and decode image from file
    int LoadImageDataFromFile(BYTE** imageData, D3D12_RESOURCE_DESC& resourceDescription, LPCWSTR filename, int& bytesPerRow)
    {
        HRESULT hr;

        // we only need one instance of the imaging factory to create decoders and frames
        static IWICImagingFactory* wicFactory;

        // reset decoder, frame and converter since these will be different for each image we load
        IWICBitmapDecoder* wicDecoder = NULL;
        IWICBitmapFrameDecode* wicFrame = NULL;
        IWICFormatConverter* wicConverter = NULL;

        bool imageConverted = false;

        if (wicFactory == NULL)
        {
            // Initialize the COM library
            CoInitialize(NULL);

            // create the WIC factory
            hr = CoCreateInstance(
                CLSID_WICImagingFactory,
                NULL,
                CLSCTX_INPROC_SERVER,
                IID_PPV_ARGS(&wicFactory)
            );
            if (FAILED(hr)) return 0;
        }

        // load a decoder for the image
        hr = wicFactory->CreateDecoderFromFilename(
            filename,                        // Image we want to load in
            NULL,                            // This is a vendor ID, we do not prefer a specific one so set to null
            GENERIC_READ,                    // We want to read from this file
            WICDecodeMetadataCacheOnLoad,    // We will cache the metadata right away, rather than when needed, which might be unknown
            &wicDecoder                      // the wic decoder to be created
        );
        if (FAILED(hr)) return 0;

        // get image from decoder (this will decode the "frame")
        hr = wicDecoder->GetFrame(0, &wicFrame);
        if (FAILED(hr)) return 0;

        // get wic pixel format of image
        WICPixelFormatGUID pixelFormat;
        hr = wicFrame->GetPixelFormat(&pixelFormat);
        if (FAILED(hr)) return 0;

        // get size of image
        UINT textureWidth, textureHeight;
        hr = wicFrame->GetSize(&textureWidth, &textureHeight);
        if (FAILED(hr)) return 0;

        // we are not handling sRGB types in this tutorial, so if you need that support, you'll have to figure
        // out how to implement the support yourself

        // convert wic pixel format to dxgi pixel format
        DXGI_FORMAT dxgiFormat = GetDXGIFormatFromWICFormat(pixelFormat);

        // if the format of the image is not a supported dxgi format, try to convert it
        if (dxgiFormat == DXGI_FORMAT_UNKNOWN)
        {
            // get a dxgi compatible wic format from the current image format
            WICPixelFormatGUID convertToPixelFormat = GetConvertToWICFormat(pixelFormat);

            // return if no dxgi compatible format was found
            if (convertToPixelFormat == GUID_WICPixelFormatDontCare) return 0;

            // set the dxgi format
            dxgiFormat = GetDXGIFormatFromWICFormat(convertToPixelFormat);

            // create the format converter
            hr = wicFactory->CreateFormatConverter(&wicConverter);
            if (FAILED(hr)) return 0;

            // make sure we can convert to the dxgi compatible format
            BOOL canConvert = FALSE;
            hr = wicConverter->CanConvert(pixelFormat, convertToPixelFormat, &canConvert);
            if (FAILED(hr) || !canConvert) return 0;

            // do the conversion (wicConverter will contain the converted image)
            hr = wicConverter->Initialize(wicFrame, convertToPixelFormat, WICBitmapDitherTypeErrorDiffusion, 0, 0, WICBitmapPaletteTypeCustom);
            if (FAILED(hr)) return 0;

            // this is so we know to get the image data from the wicConverter (otherwise we will get from wicFrame)
            imageConverted = true;
        }

        int bitsPerPixel = GetDXGIFormatBitsPerPixel(dxgiFormat); // number of bits per pixel
        bytesPerRow = (textureWidth * bitsPerPixel) / 8; // number of bytes in each row of the image data
        int imageSize = bytesPerRow * textureHeight; // total image size in bytes

        // allocate enough memory for the raw image data, and set imageData to point to that memory
        *imageData = (BYTE*)malloc(imageSize);

        // copy (decoded) raw image data into the newly allocated memory (imageData)
        if (imageConverted)
        {
            // if image format needed to be converted, the wic converter will contain the converted image
            hr = wicConverter->CopyPixels(0, bytesPerRow, imageSize, *imageData);
            if (FAILED(hr)) return 0;
        }
        else
        {
            // no need to convert, just copy data from the wic frame
            hr = wicFrame->CopyPixels(0, bytesPerRow, imageSize, *imageData);
            if (FAILED(hr)) return 0;
        }

        // now describe the texture with the information we have obtained from the image
        resourceDescription = {};
        resourceDescription.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        resourceDescription.Alignment = 0; // may be 0, 4KB, 64KB, or 4MB. 0 will let runtime decide between 64KB and 4MB (4MB for multi-sampled textures)
        resourceDescription.Width = textureWidth; // width of the texture
        resourceDescription.Height = textureHeight; // height of the texture
        resourceDescription.DepthOrArraySize = 1; // if 3d image, depth of 3d image. Otherwise an array of 1D or 2D textures (we only have one image, so we set 1)
        resourceDescription.MipLevels = 1; // Number of mipmaps. We are not generating mipmaps for this texture, so we have only one level
        resourceDescription.Format = dxgiFormat; // This is the dxgi format of the image (format of the pixels)
        resourceDescription.SampleDesc.Count = 1; // This is the number of samples per pixel, we just want 1 sample
        resourceDescription.SampleDesc.Quality = 0; // The quality level of the samples. Higher is better quality, but worse performance
        resourceDescription.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN; // The arrangement of the pixels. Setting to unknown lets the driver choose the most efficient one
        resourceDescription.Flags = D3D12_RESOURCE_FLAG_NONE; // no flags

        // return the size of the image. remember to delete the image once your done with it (in this tutorial once its uploaded to the gpu)
        return imageSize;
    }

    HRESULT CreateWICTextureFromFile12(_In_ ID3D12Device* device,
        _In_ ID3D12GraphicsCommandList* cmdList,
        _In_z_ const wchar_t* szFileName,
        _Out_ Microsoft::WRL::ComPtr<ID3D12Resource>& texture,
        _Out_ Microsoft::WRL::ComPtr<ID3D12Resource>& textureUploadHeap)
    {
        BYTE* ImageData = nullptr;
        D3D12_RESOURCE_DESC rcDescriptor = {};
        int bPerRow = 0;

        int imageSize = LoadImageDataFromFile(&ImageData, rcDescriptor, szFileName, bPerRow);


        device->CreateCommittedResource(
            &unmove(CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT)),
            D3D12_HEAP_FLAG_NONE,
            &rcDescriptor,
            D3D12_RESOURCE_STATE_COPY_DEST,
            nullptr,
            IID_PPV_ARGS(texture.GetAddressOf()));

        UINT64 textureUploaderBufferSize = 0;
        device->GetCopyableFootprints(&rcDescriptor, 0, 1, 0, nullptr, nullptr, nullptr, &textureUploaderBufferSize);

        device->CreateCommittedResource(
            &unmove(CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD)),
            D3D12_HEAP_FLAG_NONE,
            &unmove(CD3DX12_RESOURCE_DESC::Buffer(textureUploaderBufferSize)),
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(textureUploadHeap.GetAddressOf()));

        D3D12_SUBRESOURCE_DATA subRcData = {};
        subRcData.pData = &ImageData[0];
        subRcData.RowPitch = bPerRow;
        subRcData.SlicePitch = bPerRow * rcDescriptor.Height;

        UpdateSubresources(cmdList, texture.Get(), textureUploadHeap.Get(), 0, 0, 1, &subRcData);

        free(ImageData);

        return 0;
    }
}


const std::array<CD3DX12_STATIC_SAMPLER_DESC, 6> GetStaticSamplers()
{
    CD3DX12_STATIC_SAMPLER_DESC pointWrap(
        0,
        D3D12_FILTER_MIN_MAG_MIP_POINT,
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,
        0.0f,
        8);

    CD3DX12_STATIC_SAMPLER_DESC pointClamp(
        1,
        D3D12_FILTER_MIN_MAG_MIP_POINT,
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
        0.0f,
        8);

    CD3DX12_STATIC_SAMPLER_DESC linearWrap(
        2,
        D3D12_FILTER_MIN_MAG_MIP_LINEAR,
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,
        0.0f,
        8);

    CD3DX12_STATIC_SAMPLER_DESC linearClamp(
        3,
        D3D12_FILTER_MIN_MAG_MIP_LINEAR,
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
        0.0f,
        8);

    CD3DX12_STATIC_SAMPLER_DESC anisotropicWrap(
        4,
        D3D12_FILTER_ANISOTROPIC,
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,
        0.0f,
        16);

    CD3DX12_STATIC_SAMPLER_DESC anisotropicClamp(
        5,
        D3D12_FILTER_ANISOTROPIC,
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
        0.0f,
        16);

    return {
        pointWrap, pointClamp,
        linearWrap, linearClamp,
        anisotropicWrap, anisotropicClamp
    };
}

void BuildVertexArray(const std::vector<Shader::FontType>& font, void* data, const char* sentence,
    float drawX, float drawY, float scaleX, float scaleY)
{
    Shader::TextVertex* vertices = (Shader::TextVertex*)data;
    size_t numLetters = strlen(sentence);

    for (size_t i = 0, k= 0; i < numLetters; ++i, k += 4)
    {
        int letter = ((int)sentence[i] - 32);

        if (letter == 0)
        {
            drawX += 3.0f + scaleX;
        }
        else
        {
            // 0 1
            // 2 3

            // top left
            vertices[k + 0].Position = DirectX::XMFLOAT3(drawX, drawY, 0.0f);
            vertices[k + 0].TexCoord = DirectX::XMFLOAT2(font[letter].left, 0.0f);

            // top right
            vertices[k + 1].Position = DirectX::XMFLOAT3(drawX + font[letter].size + scaleX, drawY, 0.0f);
            vertices[k + 1].TexCoord = DirectX::XMFLOAT2(font[letter].right, 0.0f);

            // bottom left
            vertices[k + 2].Position = DirectX::XMFLOAT3(drawX, drawY - scaleY, 0.0f);
            vertices[k + 2].TexCoord = DirectX::XMFLOAT2(font[letter].left, 0.8f);

            // bottom right
            vertices[k + 3].Position = DirectX::XMFLOAT3(drawX + font[letter].size + scaleX, drawY - scaleY, 0.0f);
            vertices[k + 3].TexCoord = DirectX::XMFLOAT2(font[letter].right, 0.8f);

            drawX += font[letter].size + 1.0f + scaleX;
        }
    }
}