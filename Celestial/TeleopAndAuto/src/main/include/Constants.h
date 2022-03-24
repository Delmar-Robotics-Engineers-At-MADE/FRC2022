#pragma once

  enum DriveSysTargetingState {
    kDriveUnknownState = 0,
    kDriveRotatingToTarget,
    kDriveOnTarget,
    kDriveNotTargeting
  };