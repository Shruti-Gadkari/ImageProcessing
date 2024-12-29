#include <cuda_runtime.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

#define BLOCK_SIZE 16

// Kernel for rotating the image
__global__ void rotateImageKernel(const unsigned char* input, unsigned char* output,
    int width, int height, float cos_theta, float sin_theta,
    int centerX, int centerY)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < width && y < height)
    {
        // Compute the rotated coordinates relative to center
        int srcX = static_cast<int>((x - centerX) * cos_theta + (y - centerY) * sin_theta + centerX);
        int srcY = static_cast<int>((y - centerY) * cos_theta - (x - centerX) * sin_theta + centerY);

        // Check if source coordinates are within bounds
        if (srcX >= 0 && srcX < width && srcY >= 0 && srcY < height)
        {
            output[y * width + x] = input[srcY * width + srcX];
        }
        else
        {
            output[y * width + x] = 0; // Set out-of-bound pixels to black
        }
    }
}

// Function to invoke kernel
void rotateImage(const unsigned char* input, unsigned char* output, int width, int height, float theta)
{
    unsigned char* d_input, * d_output;
    size_t imageSize = width * height * sizeof(unsigned char);

    // Allocate device memory
    cudaMalloc(&d_input, imageSize);
    cudaMalloc(&d_output, imageSize);

    // Copy input data to device
    cudaMemcpy(d_input, input, imageSize, cudaMemcpyHostToDevice);

    // Calculate rotation matrix values
    float radians = theta * M_PI / 180.0;
    float cos_theta = cosf(radians);
    float sin_theta = sinf(radians);

    int centerX = width / 2;
    int centerY = height / 2;

    // Configure kernel launch parameters
    dim3 blockDim(BLOCK_SIZE, BLOCK_SIZE);
    dim3 gridDim((width + BLOCK_SIZE - 1) / BLOCK_SIZE, (height + BLOCK_SIZE - 1) / BLOCK_SIZE);

    // Launch kernel
    rotateImageKernel << <gridDim, blockDim >> > (d_input, d_output, width, height, cos_theta, sin_theta, centerX, centerY);

    // Copy result back to host
    cudaMemcpy(output, d_output, imageSize, cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree(d_input);
    cudaFree(d_output);
}

// Function to read PGM file
bool readPGM(const std::string& filename, unsigned char*& data, int& width, int& height)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open input file." << std::endl;
        return false;
    }

    std::string magicNumber;
    file >> magicNumber;
    if (magicNumber != "P5") {
        std::cerr << "Error: Unsupported file format." << std::endl;
        return false;
    }

    file >> width >> height;
    int maxVal;
    file >> maxVal;
    file.ignore(1); // Ignore the newline character

    // Debugging: Log dimensions and maxVal
    std::cout << "Width: " << width << ", Height: " << height << ", Max Value: " << maxVal << std::endl;

    size_t imageSize = width * height;
    data = new unsigned char[imageSize];
    file.read(reinterpret_cast<char*>(data), imageSize);

#if 0
    if (file.gcount() != imageSize) {
		printf("file.gcount() = %d, imageSize = %d\n", file.gcount(), imageSize);
        std::cerr << "Error: Unexpected end of file while reading pixel data." << std::endl;
        delete[] data;
        return false;
    }
#endif


    file.close();
    return true;
}

// Function to write PGM file
void writePGM(const std::string& filename, const unsigned char* data, int width, int height)
{
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open output file." << std::endl;
        return;
    }

    // Write PGM header
    file << "P5\n" << width << " " << height << "\n255\n";
    file.write(reinterpret_cast<const char*>(data), width * height);

    file.close();
}

int main()
{
    std::string inputFile = "lena.pgm";
    std::string outputFile = "rotated_output.pgm";

    unsigned char* inputImage = nullptr;
    int width, height;

    if (!readPGM(inputFile, inputImage, width, height)) {
        return -1;
    }

	printf("width = %d, height = %d\n", width, height);

    float theta = 45.0; // Rotation angle in degrees

    unsigned char* outputImage = new unsigned char[width * height];

    // Debugging: Log output dimensions
    std::cout << "Output Dimensions: " << width << "x" << height << std::endl;

    // Rotate the image
    //rotateImage(inputImage, outputImage, width, height, theta);

    // Save the rotated image
   // writePGM(outputFile, outputImage, width, height);
    writePGM(outputFile, inputImage, width, height);

    std::cout << "Image rotation completed and saved as " << outputFile << "!" << std::endl;

    // Cleanup
    delete[] inputImage;
    delete[] outputImage;

    return 0;
}
