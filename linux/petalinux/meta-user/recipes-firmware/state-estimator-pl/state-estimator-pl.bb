SUMMARY = "State-estimator KR260 programmable-logic firmware"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

inherit dfx_dtg_zynqmp_full

SRC_URI = "file://state_estimator_kr260.xsa"

COMPATIBLE_MACHINE:zynqmp = "zynqmp"
