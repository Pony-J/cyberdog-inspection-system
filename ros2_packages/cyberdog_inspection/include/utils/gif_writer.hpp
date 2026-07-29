#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>
#include "utils/gif.h"

namespace utils
{

class GifWriterWrapper{

public:
    GifWriterWrapper() = default;
    ~GifWriterWrapper() = default;

    //initialize the gif writer
    bool begin(const std::string& filename, int width, int height, int delay = 10) {
        // create directory if not exists
        std::filesystem::path filepath(filename);
        std::filesystem::path parent = filepath.parent_path();
        if(!parent.empty() && !std::filesystem::exists(parent)) {
            std::filesystem::create_directories(parent);
        }

        width_ = width;
        height_ = height;
        delay_ = delay;
        return GifBegin(&writer_, filename.c_str(), width, height, delay);
    }

    //add a frame to the gif
    bool addFrame(const cv::Mat& frame) {
        if (frame.empty()) return false;

        cv::Mat resized;
        if (frame.cols != width_ || frame.rows != height_) {
            cv::resize(frame, resized, cv::Size(width_, height_));
        } else {
            resized = frame;
        }

        cv::Mat rgba;
        if (resized.channels() == 3) {
            cv::cvtColor(resized, rgba, cv::COLOR_BGR2RGBA);
        } else if (resized.channels() == 4) {
            cv::cvtColor(resized, rgba, cv::COLOR_BGRA2RGBA);
        } else if (resized.channels() == 1) {
            cv::cvtColor(resized, rgba, cv::COLOR_GRAY2RGBA);
        } else {
            return false;
        }

        return GifWriteFrame(&writer_, rgba.data, width_, height_, delay_);
    }

    //save the gif
    bool end() {
        return GifEnd(&writer_);
    }

private:
    GifWriter writer_;
    int width_ = 0;
    int height_ = 0;
    int delay_ = 10;
};

} // namespace utils
