#include <OpenNI.h>
#include <iostream>
#include <vector>
#include <csignal>

using namespace openni;

// ---------- Utilities ----------
#define CHECK_RC(rc, what)                                   \
    if ((rc) != STATUS_OK) {                                 \
        std::cerr << what << " failed: "                     \
                  << OpenNI::getExtendedError() << std::endl;\
        return 1;                                            \
    }

static bool running = true;

void signalHandler(int)
{
    running = false;
}

// ---------- Main ----------
int main()
{
    // Handle Ctrl+C cleanly
    signal(SIGINT, signalHandler);

    // ---- Initialize OpenNI ----
    Status rc = OpenNI::initialize();
    CHECK_RC(rc, "OpenNI initialize");

    Device device;
    rc = device.open(ANY_DEVICE);
    CHECK_RC(rc, "Device open");

    // Optional but recommended: align depth to color
    device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);

    VideoStream depthStream, colorStream, irStream;
    std::vector<VideoStream*> streams;

    // ---- Depth ----
    if (device.hasSensor(SENSOR_DEPTH)) {
        rc = depthStream.create(device, SENSOR_DEPTH);
        CHECK_RC(rc, "Create depth stream");
        depthStream.start();
        streams.push_back(&depthStream);
        std::cout << "Depth stream started\n";
    }

    // ---- Color ----
    if (device.hasSensor(SENSOR_COLOR)) {
        rc = colorStream.create(device, SENSOR_COLOR);
        CHECK_RC(rc, "Create color stream");
        colorStream.start();
        streams.push_back(&colorStream);
        std::cout << "Color stream started\n";
    }

    // ---- IR ----
    if (device.hasSensor(SENSOR_IR)) {
        rc = irStream.create(device, SENSOR_IR);
        CHECK_RC(rc, "Create IR stream");
        irStream.start();
        streams.push_back(&irStream);
        std::cout << "IR stream started\n";
    }

    if (streams.empty()) {
        std::cerr << "No valid streams available\n";
        device.close();
        OpenNI::shutdown();
        return 1;
    }

    std::cout << "Astra driver running (Ctrl+C to exit)\n";

    // ---------- Main capture loop ----------
    while (running) {
        int readyIndex = -1;

        rc = OpenNI::waitForAnyStream(
            streams.data(),
            streams.size(),
            &readyIndex,
            2000   // timeout in ms
        );

        if (rc != STATUS_OK) {
            std::cerr << "waitForAnyStream failed: "
                      << OpenNI::getExtendedError()
                      << std::endl;
            continue;
        }

        VideoFrameRef frame;
        streams[readyIndex]->readFrame(&frame);

        if (!frame.isValid())
            continue;

        SensorType type =
            streams[readyIndex]->getSensorInfo().getSensorType();

        switch (type) {

            case SENSOR_DEPTH:
                std::cout << "[DEPTH] "
                          << frame.getWidth() << "x"
                          << frame.getHeight()
                          << " ts=" << frame.getTimestamp()
                          << std::endl;
                break;

            case SENSOR_COLOR:
                std::cout << "[COLOR] "
                          << frame.getWidth() << "x"
                          << frame.getHeight()
                          << " ts=" << frame.getTimestamp()
                          << std::endl;
                break;

            case SENSOR_IR:
                std::cout << "[IR] "
                          << frame.getWidth() << "x"
                          << frame.getHeight()
                          << " ts=" << frame.getTimestamp()
                          << std::endl;
                break;

            default:
                break;
        }
    }

    // ---------- Clean shutdown ----------
    std::cout << "\nShutting down streams...\n";

    if (depthStream.isValid()) depthStream.stop();
    if (colorStream.isValid()) colorStream.stop();
    if (irStream.isValid())    irStream.stop();

    depthStream.destroy();
    colorStream.destroy();
    irStream.destroy();

    device.close();
    OpenNI::shutdown();

    std::cout << "Shutdown complete\n";
    return 0;
}
