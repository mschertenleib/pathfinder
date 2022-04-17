#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "lodepng.h"

int main(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cerr << "Usage: " << argv[0] << " <filename> <width> <height>\n";
        return EXIT_FAILURE;
    }

    const auto filename = argv[1];
    const unsigned int width = std::stoul(argv[2]);
    const unsigned int height = std::stoul(argv[3]);
    const auto num_pixels = width * height;
    const auto buffer_size = num_pixels * 2;

    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file \"" << filename << "\"\n";
        return EXIT_FAILURE;
    }
    const auto file_size = std::filesystem::file_size(filename);
    if (file_size != buffer_size)
    {
        std::cerr << "Unexpected file size\n";
        return EXIT_FAILURE;
    }

    std::vector<char> buffer(buffer_size);
    file.read(buffer.data(), buffer_size);
    file.close();

    std::vector<std::uint8_t> image(num_pixels * 3);

    for (unsigned int i {0}; i < height; ++i)
    {
        for (unsigned int j {0}; j < width; ++j)
        {
            const auto pixel_index = i * width + j;
            const auto msb = static_cast<std::uint8_t>(buffer[pixel_index * 2]);
            const auto lsb =
                static_cast<std::uint8_t>(buffer[pixel_index * 2 + 1]);
            const auto rgb565 = static_cast<std::uint16_t>((msb << 8) | lsb);

            const auto r5 = (rgb565 & 0b1111'1000'0000'0000u) >> 11;
            const auto g6 = (rgb565 & 0b0000'0111'1110'0000u) >> 5;
            const auto b5 = (rgb565 & 0b0000'0000'0001'1111u) >> 0;

            const auto rp = static_cast<float>(r5) / 31.0f;
            const auto gp = static_cast<float>(g6) / 63.0f;
            const auto bp = static_cast<float>(b5) / 31.0f;

            const auto r8 = static_cast<unsigned char>(rp * 255.0f);
            const auto g8 = static_cast<unsigned char>(gp * 255.0f);
            const auto b8 = static_cast<unsigned char>(bp * 255.0f);

            image[pixel_index * 3 + 0] = r8;
            image[pixel_index * 3 + 1] = g8;
            image[pixel_index * 3 + 2] = b8;
        }
    }

    const auto error =
        lodepng::encode("capture.png", image, width, height, LCT_RGB, 8);
    if (error)
    {
        std::cerr << lodepng_error_text(error);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}