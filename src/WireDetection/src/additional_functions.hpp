//in this code Photoneo examples were used

#pragma once
#define PHOXI_PCL_SUPPORT
#include "PhoXi.h"
#include <iostream>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <unistd.h>
#include <vector>

void printDeviceInfo(const pho::api::PhoXiDeviceInformation &DeviceInfo) {
    std::cout << "  Name:                    " << DeviceInfo.Name << std::endl;
    std::cout << "  Hardware Identification: " << DeviceInfo.HWIdentification
              << std::endl;
    std::cout << "  Type:                    " << std::string(DeviceInfo.Type)
              << std::endl;
    std::cout << "  Firmware version:        " << DeviceInfo.FirmwareVersion
              << std::endl;
    std::cout << "  Status:                  "
              << (DeviceInfo.Status.Attached
                  ? "Attached to PhoXi Control. "
                  : "Not Attached to PhoXi Control. ")
              << (DeviceInfo.Status.Ready ? "Ready to connect" : "Occupied")
              << std::endl
              << std::endl;
}

void printDeviceInfoList(
        const std::vector<pho::api::PhoXiDeviceInformation> &DeviceList) {
    for (std::size_t i = 0; i < DeviceList.size(); ++i) {
        std::cout << "Device: " << i << std::endl;
        printDeviceInfo(DeviceList[i]);
    }
}

void printFrameInfo(const pho::api::PFrame &Frame) {
    const pho::api::FrameInfo &FrameInfo = Frame->Info;
    std::cout << "  Frame params: " << std::endl;
    std::cout << "    Frame Index: " << FrameInfo.FrameIndex << std::endl;
    std::cout << "    Frame Timestamp: " << FrameInfo.FrameTimestamp << " s"
              << std::endl;
    std::cout << "    Frame Acquisition duration: " << FrameInfo.FrameDuration
              << " ms" << std::endl;
    std::cout << "    Frame Computation duration: "
              << FrameInfo.FrameComputationDuration << " ms" << std::endl;
    std::cout << "    Frame Transfer duration: "
              << FrameInfo.FrameTransferDuration << " ms" << std::endl;
    std::cout << "    Sensor Position: [" << FrameInfo.SensorPosition.x << "; "
              << FrameInfo.SensorPosition.y << "; "
              << FrameInfo.SensorPosition.z << "]" << std::endl;
}

void printFrameData(const pho::api::PFrame &Frame) {
    if (Frame->Empty()) {
        std::cout << "Frame is empty.";
        return;
    }
    std::cout << "  Frame data: " << std::endl;
    if (!Frame->PointCloud.Empty()) {
        std::cout << "    PointCloud:    (" << Frame->PointCloud.Size.Width
                  << " x " << Frame->PointCloud.Size.Height
                  << ") Type: " << Frame->PointCloud.GetElementName()
                  << std::endl;
    }
    if (!Frame->NormalMap.Empty()) {
        std::cout << "    NormalMap:     (" << Frame->NormalMap.Size.Width
                  << " x " << Frame->NormalMap.Size.Height
                  << ") Type: " << Frame->NormalMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->DepthMap.Empty()) {
        std::cout << "    DepthMap:      (" << Frame->DepthMap.Size.Width
                  << " x " << Frame->DepthMap.Size.Height
                  << ") Type: " << Frame->DepthMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->ConfidenceMap.Empty()) {
        std::cout << "    ConfidenceMap: (" << Frame->ConfidenceMap.Size.Width
                  << " x " << Frame->ConfidenceMap.Size.Height
                  << ") Type: " << Frame->ConfidenceMap.GetElementName()
                  << std::endl;
    }
    if (!Frame->Texture.Empty()) {
        std::cout << "    Texture:       (" << Frame->Texture.Size.Width
                  << " x " << Frame->Texture.Size.Height
                  << ") Type: " << Frame->Texture.GetElementName() << std::endl;
    }
}

pho::api::PPhoXi create_PphoXi(){

    pho::api::PhoXiFactory Factory;

    // Check if the PhoXi Control Software is running
    if (!Factory.isPhoXiControlRunning()) {
        std::cout << "PhoXi Control Software is not running" << std::endl;
        return 0;
    }

    // Get List of available devices on the network
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList =
            Factory.GetDeviceList();
    if (DeviceList.empty()) {
        std::cout << "PhoXi Factory has found 0 devices" << std::endl;
        return 0;
    }
    printDeviceInfoList(DeviceList);

    // Try to connect device opened in PhoXi Control, if any
    pho::api::PPhoXi PhoXiDevice = Factory.CreateAndConnectFirstAttached();
    if (PhoXiDevice) {
        std::cout << "You have already PhoXi device opened in PhoXi Control, "
                     "the API Example is connected to device: "
                  << (std::string) PhoXiDevice->HardwareIdentification
                  << std::endl;
    } else {
        std::cout
                << "You have no PhoXi device opened in PhoXi Control, the API ";
        for (size_t i = 0; i < DeviceList.size(); i++) {
            std::cout << "Example will try to connect to ..."
                      << DeviceList.at(i).HWIdentification << std::endl;
            // wait 5 second for scanner became ready
            PhoXiDevice = Factory.CreateAndConnect(
                    DeviceList.at(i).HWIdentification, 5000);
            if (PhoXiDevice) {
                std::cout << "succesfully connected" << std::endl;
                break;
            }
            if (i == DeviceList.size() - 1) {
                std::cout << "Can not connect to any device" << std::endl;
            }
        }
    }

    // Check if device was created
    if (!PhoXiDevice) {
        std::cout << "Your device was not created!" << std::endl;
        return 0;
    }

    // Check if device is connected
    if (!PhoXiDevice->isConnected()) {
        std::cout << "Your device is not connected" << std::endl;
        return 0;
    }
    return PhoXiDevice;
}

pho::api::PFrame startSoftwareTriggerExample(pho::api::PPhoXi &PhoXiDevice) {
    pho::api::PFrame Frame;
    if (PhoXiDevice->isAcquiring()) {
        // Stop acquisition to change trigger mode
        PhoXiDevice->StopAcquisition();
    }

    PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
    std::cout << "Software trigger mode was set" << std::endl;
    PhoXiDevice->ClearBuffer();
    PhoXiDevice->StartAcquisition();
    if (!PhoXiDevice->isAcquiring()) {
        std::cout << "Your device could not start acquisition!" << std::endl;
// return;
    }

    for (int i = 0; i < 20; ++i) {
        std::cout << "Triggering the " << i << "-th frame" << std::endl;
        int FrameID = PhoXiDevice->TriggerFrame();
        if (FrameID < 0) {
            // If negative number is returned trigger was unsuccessful
            std::cout << "Trigger was unsuccessful!" << std::endl;
            continue;
        } else {
            std::cout << "Frame was triggered, Frame Id: " << FrameID
                      << std::endl;
        }

        std::cout << "Waiting for frame " << i << std::endl;
        pho::api::PFrame Frame = PhoXiDevice->GetSpecificFrame(
                FrameID, pho::api::PhoXiTimeout::Infinity);

        if (Frame) {
            printFrameInfo(Frame);
            printFrameData(Frame);
            return Frame;

        } else {
            std::cout << "Failed to retrieve the frame!" << std::endl;
            return Frame;
        }
    }
    return Frame;
}



