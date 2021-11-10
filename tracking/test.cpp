// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Edmund Stoiber, German Aerospace Center (DLR)

#include <tracking.h>

#include <thread>
#include <iostream>


int main() {

    startTracking();
    while (true) {
        auto obj = getData().object0;
        auto objM = getMarker();
        cv::Point3_ p1(obj.tvec.pos0, obj.tvec.pos1, obj.tvec.pos2);
        cv::Point3_ p2(objM.tvec.pos0, objM.tvec.pos1, objM.tvec.pos2);
        auto dist = cv::norm(p1-p2);
        std::cout << dist <<std::endl;

        _sleep(1000);
    }

}
