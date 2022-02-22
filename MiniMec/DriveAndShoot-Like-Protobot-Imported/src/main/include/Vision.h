#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/Timer.h>

class VisionSubsystem {
private:
    frc::Timer m_timer;
    units::time::second_t timeBallsLastSeen = 0_s;
public:
    double distanceClosestBall = 0.0;
    double angleClosestBall = 0.0;

    class Gamepiece{
    public:
        double distance = 0, xOffset = 0;
        virtual double getAngle();
    };

    class Ball : public Gamepiece {
    private:
        static constexpr double kGamepieceHeightInch = 7.0;
    public:
        Ball(std::vector<double> &box);
    };

    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("datatable");
    int totalObjects = 0, totalBalls = 0;
    // std::vector<Ball*> balls = std::vector<Ball*>(0);  // this is an unnecessary memory management complexity
    std::vector<std::string> classes;
    std::vector<double> boxes, box;
    nt::NetworkTableEntry totalObjectsEntry, classesEntry, boxesEntry; // were pointers

    virtual ~VisionSubsystem() { // destructor
        //delete table;
        //delete totalObjectsEntry;
        //delete classesEntry;
        //delete boxesEntry;
    }
    VisionSubsystem(); // constructor

    /**
     * Periodically updates the list of detected objects with the data found on
     * NetworkTables Also creates array of cargo and their relative position.
     */
    void periodic(); // was override

    virtual int getTotalBalls();
    virtual std::vector<Ball*> getBalls();
    virtual void disposeBalls(std::vector<Ball*> balls);
    virtual void updateClosestBall();
};
