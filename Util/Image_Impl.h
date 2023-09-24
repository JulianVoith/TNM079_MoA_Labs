#pragma once

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stb_image_write.h>
#include <stb_image.h>

template <typename DataType>
Image<DataType>::Image(const std::string& filename) : Image() {
    // Try to read the file
    this->ReadFromFile(filename);
}

template <typename DataType>
Image<DataType>::Image(const Image& img) : Image(img.mWidth, img.mHeight, img.mComponents) {
    std::copy(std::begin(img.mData), std::end(img.mData), std::begin(mData));
}

template <typename DataType>
void Image<DataType>::SetDimensions(GLuint width, GLuint height, GLuint components) {
    mWidth = width;
    mHeight = height;
    mComponents = components;

    mData = std::vector<DataType>(mWidth * mHeight * mComponents);
}

template <typename DataType>
void Image<DataType>::ReadFromFile(const std::string& filename) {
    int width;
    int height;
    int components;
    unsigned char* imageData = stbi_load(filename.c_str(), &width, &height, &components, 0);

    if (imageData) {
        this->SetDimensions(width, height, components);

        const auto imageSize = mWidth * mHeight * mComponents;

        std::transform(imageData, imageData + imageSize, std::begin(mData),
                       [](const auto val) { return static_cast<DataType>(val) / DataType{255}; });

        stbi_image_free(imageData);
    } else {
        std::cerr << "Could not read PNG file " << filename << std::endl;
        std::cerr << stbi_failure_reason() << std::endl;
    }
}

template <typename DataType>
void Image<DataType>::SaveToFile(const std::string& filename) {
    std::vector<GLubyte> data(mData.size());

    std::transform(std::begin(mData), std::end(mData), std::begin(data),
                   [](const auto val) { return static_cast<GLubyte>(val * DataType{255}); });

    stbi_flip_vertically_on_write(1);

    if (!stbi_write_png(filename.c_str(), mWidth, mHeight, mComponents, data.data(),
                        static_cast<int>(mWidth * mComponents * sizeof(GLubyte)))) {
        std::cerr << "Could not write PNG file." << std::endl;
        return;
    }

    std::cout << "PNG file written to '" << filename << "'" << std::endl << std::endl;
}

template <typename DataType>
void Image<DataType>::CaptureScreen() {
    // Get size of the window
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    this->SetDimensions(viewport[2], viewport[3], 4);

    glFinish();

    std::vector<GLubyte> data(viewport[2] * viewport[3] * 4);

    glReadPixels(0, 0, mWidth, mHeight, GL_RGBA, GL_UNSIGNED_BYTE, data.data());

    std::transform(std::begin(data), std::end(data), std::begin(mData),
                   [](const auto val) { return static_cast<DataType>(val) / DataType{255}; });
}

template <typename DataType>
GLuint Image<DataType>::LoadTexture() {
    // This is translated from if(mData==NULL) but couldn't we just resize here?
    if (mData.empty()) return 0;

    // OpenGL can't handle 2 component images
    GLuint components = (mComponents != 2 ? mComponents : 3);

    // We load the texture as a float image
    GLuint imageSize = mWidth * mHeight * components;
    std::vector<GLubyte> data(imageSize);

    std::copy(std::begin(mData), std::end(mData), data);

    GLuint texID;

    if (glIsTexture(mTexID) == GL_FALSE) glGenTextures(1, &mTexID);

    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mTexID);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, components, mWidth, mHeight, 0,
                 (components == 1 ? GL_LUMINANCE : (components == 3 ? GL_RGB : GL_RGBA)), GL_FLOAT,
                 data.data());

    fprintf(stderr, "Image uploaded to texture memory [ID: %i]\n", mTexID);

    return mTexID;
}

template <typename DataType>
DataType Image<DataType>::Max(int component) {
    return *std::max_element(std::begin(mData), std::end(mData));
}

template <typename DataType>
DataType Image<DataType>::Min(int component) {
    return *std::min_element(std::begin(mData), std::end(mData));
}

template <typename DataType>
Image<DataType>& Image<DataType>::operator=(const Image& img) {
    if (this == &img) return *this;

    mWidth = img.mWidth;
    mHeight = img.mHeight;
    mComponents = img.mComponents;

    mData = img.mData;

    return *this;
}
