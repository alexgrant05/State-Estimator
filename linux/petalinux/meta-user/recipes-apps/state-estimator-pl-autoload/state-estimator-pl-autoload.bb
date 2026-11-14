SUMMARY = "Load and verify the state-estimator PL during Linux startup"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

inherit systemd

SRC_URI = " \
    file://state-estimator-pl-load.sh \
    file://state-estimator-pl-load.service \
"

RDEPENDS:${PN} = "bash fpga-manager-script"

SYSTEMD_SERVICE:${PN} = "state-estimator-pl-load.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"

do_install() {
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/state-estimator-pl-load.sh \
        ${D}${sbindir}/state-estimator-pl-load
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/state-estimator-pl-load.service \
        ${D}${systemd_system_unitdir}/state-estimator-pl-load.service
}

FILES:${PN} += "${systemd_system_unitdir}/state-estimator-pl-load.service"
