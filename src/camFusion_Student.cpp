
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor,
                    cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT) {
    // Remove outliers with a standard deviation filter
    double mean_x = 0, mean_y = 0, mean_z = 0, stddev_x = 0, stddev_y = 0, stddev_z = 0;
    int nPoints = lidarPoints.size();
    for (auto &point: lidarPoints) {
        mean_x += point.x;
        mean_y += point.y;
        mean_z += point.z;
    }
    mean_x /= nPoints;
    mean_y /= nPoints;
    mean_z /= nPoints;
    for (auto &point: lidarPoints) {
        stddev_x += (point.x - mean_x) * (point.x - mean_x);
        stddev_y += (point.y - mean_y) * (point.y - mean_y);
        stddev_z += (point.z - mean_z) * (point.z - mean_z);
    }
    stddev_x = sqrt(stddev_x / nPoints);
    stddev_y = sqrt(stddev_y / nPoints);
    stddev_z = sqrt(stddev_z / nPoints);
    lidarPoints.erase(
        remove_if(lidarPoints.begin(), lidarPoints.end(), [&](LidarPoint const &point) {
            return (abs(point.x - mean_x) > 2 * stddev_x) ||
                   (abs(point.y - mean_y) > 2 * stddev_y) ||
                   (abs(point.z - mean_z) > 2 * stddev_z);
        }),
        lidarPoints.end());

    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1) {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2) {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt)) {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1) {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait) {
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1) {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2) {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int) it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i) {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if (bWait) {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              const std::vector<cv::KeyPoint> &kptsPrev,
                              const std::vector<cv::KeyPoint> &kptsCurr,
                              const std::vector<cv::DMatch> &kptMatches) {
    std::vector<double> pairwiseDistances;
    std::vector<cv::DMatch> validMatches;
    for (const auto &match: kptMatches) {
        auto &currentKeypoint = kptsCurr[match.trainIdx];
        auto &previousKeypoint = kptsPrev[match.queryIdx];
        if (boundingBox.roi.contains(currentKeypoint.pt)) {
            double pairwiseDist = cv::norm(currentKeypoint.pt - previousKeypoint.pt);
            pairwiseDistances.push_back(pairwiseDist);
            validMatches.push_back(match);
        }
    }
    int numPairs = pairwiseDistances.size();
    double meanPairwiseDist = std::accumulate(pairwiseDistances.begin(), pairwiseDistances.end(), 0.0) / numPairs;
    double distanceThreshold = meanPairwiseDist * 1.3;
    for (int i = 0; i < numPairs; i++) {
        if (pairwiseDistances[i] < distanceThreshold) {
            boundingBox.keypoints.push_back(kptsCurr[validMatches[i].trainIdx]);
            boundingBox.kptMatches.push_back(validMatches[i]);
        }
    }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev,
                      std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> &kptMatches,
                      double frameRate, double &TTC, cv::Mat *visImg) {
    std::vector<double> distRatios;
    for (auto outerIt = kptMatches.begin(); outerIt != kptMatches.end() - 1; ++outerIt) {
        const cv::KeyPoint &outerCurrKpt = kptsCurr.at(outerIt->trainIdx);
        const cv::KeyPoint &outerPrevKpt = kptsPrev.at(outerIt->queryIdx);
        for (auto innerIt = std::next(outerIt); innerIt != kptMatches.end(); ++innerIt) {
            const double minDist = 100.0;
            const cv::KeyPoint &innerCurrKpt = kptsCurr.at(innerIt->trainIdx);
            const cv::KeyPoint &innerPrevKpt = kptsPrev.at(innerIt->queryIdx);
            double currDist = cv::norm(outerCurrKpt.pt - innerCurrKpt.pt);
            double prevDist = cv::norm(outerPrevKpt.pt - innerPrevKpt.pt);
            if (prevDist > std::numeric_limits<double>::epsilon() && currDist >= minDist) {
                double distRatio = currDist / prevDist;
                distRatios.push_back(distRatio);
            }
        }
    }
    if (distRatios.empty()) {
        TTC = NAN;
        return;
    }
    std::sort(distRatios.begin(), distRatios.end());
    const long medianIndex = static_cast<long>(distRatios.size() / 2.0);
    const bool isEven = distRatios.size() % 2 == 0;
    const double medianDistRatio = isEven ? (distRatios[medianIndex - 1] + distRatios[medianIndex]) / 2.0
                                          : distRatios[medianIndex];
    const double deltaT = 1 / frameRate;
    TTC = -deltaT / (1 - medianDistRatio);
}

double computeMedian(std::vector<double> &sortedVec) {
    size_t size = sortedVec.size();
    if (size % 2 == 0) {
        return (sortedVec[size / 2 - 1] + sortedVec[size / 2]) / 2;
    } else {
        return sortedVec[size / 2];
    }
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC) {
    double dT = 1 / frameRate;
    double edgeCase = 2.0;
    auto isInsideLane = [&edgeCase](const LidarPoint &lidar_p) {
        return abs(lidar_p.y) < edgeCase;
    };
    std::vector<double> lidarPointsCurrX, lidarPointsPrevX;
    for (const auto &point: lidarPointsPrev) {
        if (isInsideLane(point)) {
            lidarPointsPrevX.push_back(point.x);
        }
    }
    for (const auto &point: lidarPointsCurr) {
        if (isInsideLane(point)) {
            lidarPointsCurrX.push_back(point.x);
        }
    }
    std::sort(lidarPointsPrevX.begin(), lidarPointsPrevX.end());
    std::sort(lidarPointsCurrX.begin(), lidarPointsCurrX.end());
    TTC = computeMedian(lidarPointsCurrX) * dT /
          (computeMedian(lidarPointsPrevX) - computeMedian(lidarPointsCurrX));
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame,
                        DataFrame &currFrame) {
    const int prevSize = prevFrame.boundingBoxes.size();
    const int currSize = currFrame.boundingBoxes.size();
    std::vector<std::vector<int>> scores(prevSize, std::vector<int>(currSize, 0));
    for (const auto &match: matches) {
        cv::Point2f prevPoint = prevFrame.keypoints[match.queryIdx].pt;
        cv::Point2f currPoint = currFrame.keypoints[match.trainIdx].pt;
        std::vector<int> prevBoxIds, currBoxIds;
        prevBoxIds.reserve(prevSize);
        currBoxIds.reserve(currSize);
        for (const auto &boundingBox: prevFrame.boundingBoxes) {
            if (boundingBox.roi.contains(prevPoint)) {
                prevBoxIds.emplace_back(boundingBox.boxID);
            }
        }
        for (const auto &boundingBox: currFrame.boundingBoxes) {
            if (boundingBox.roi.contains(currPoint)) {
                currBoxIds.emplace_back(boundingBox.boxID);
            }
        }
        for (const auto &prevBoxId: prevBoxIds) {
            for (const auto &currBoxId: currBoxIds) {
                scores[prevBoxId][currBoxId]++;
            }
        }
    }
    for (int prevBox = 0; prevBox < prevSize; prevBox++) {
        auto &score = scores[prevBox];
        int currBoxId = std::max_element(score.begin(), score.end()) - score.begin();
        int maxMatchingScore = score[currBoxId];
        if (maxMatchingScore > 0) {
            bbBestMatches.emplace(prevBox, currBoxId);
        }
    }
}
