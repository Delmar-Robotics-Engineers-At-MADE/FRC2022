#pragma once

  enum DriveSysTargetingState {
    kDriveUnknownState = 0,
    kDriveWaitingForTarget,
    kDriveRotatingToTarget,
    kDriveOnTarget,
    kDriveNotTargeting
  };

  static const int kTimeoutMs = 30;