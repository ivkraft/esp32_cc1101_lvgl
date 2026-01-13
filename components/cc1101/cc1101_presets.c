#include "cc1101_presets.h"
#include "cc1101_regs.h"


const uint8_t subghz_device_cc1101_preset_2fsk_dev2_38khz_async_regs[] = {

    /* GPIO GD0 */
    CC1101_IOCFG0,
    0x0D, // GD0 as async serial data output/input

    /* Frequency Synthesizer Control */
    CC1101_FSCTRL1,
    0x06, // IF = (26*10^6) / (2^10) * 0x06 = 152343.75Hz

    /* Packet engine */
    CC1101_PKTCTRL0,
    0x32, // Async, continious, no whitening
    CC1101_PKTCTRL1,
    0x04,

    // // Modem Configuration
    CC1101_MDMCFG0,
    0x00,
    CC1101_MDMCFG1,
    0x02,
    CC1101_MDMCFG2,
    0x04, // Format 2-FSK/FM, No preamble/sync, Disable (current optimized)
    CC1101_MDMCFG3,
    0x83, // Data rate is 4.79794 kBaud
    CC1101_MDMCFG4,
    0x67, //Rx BW filter is 270.833333 kHz
    CC1101_DEVIATN,
    0x04, //Deviation 2.380371 kHz

    /* Main Radio Control State Machine */
    CC1101_MCSM0,
    0x18, // Autocalibrate on idle-to-rx/tx, PO_TIMEOUT is 64 cycles(149-155us)

    /* Frequency Offset Compensation Configuration */
    CC1101_FOCCFG,
    0x16, // no frequency offset compensation, POST_K same as PRE_K, PRE_K is 4K, GATE is off

    /* Automatic Gain Control */
    CC1101_AGCCTRL0,
    0x91, //10 - Medium hysteresis, medium asymmetric dead zone, medium gain ; 01 - 16 samples agc; 00 - Normal AGC, 01 - 8dB boundary
    CC1101_AGCCTRL1,
    0x00, // 0; 0 - LNA 2 gain is decreased to minimum before decreasing LNA gain; 00 - Relative carrier sense threshold disabled; 0000 - RSSI to MAIN_TARGET
    CC1101_AGCCTRL2,
    0x07, // 00 - DVGA all; 000 - MAX LNA+LNA2; 111 - MAIN_TARGET 42 dB

    /* Wake on radio and timeouts control */
    CC1101_WORCTRL,
    0xFB, // WOR_RES is 2^15 periods (0.91 - 0.94 s) 16.5 - 17.2 hours

    /* Frontend configuration */
    CC1101_FREND0,
    0x10, // Adjusts current TX LO buffer
    CC1101_FREND1,
    0x56,

    /* End load reg */
    0,
    0,

    // 2fsk_async_patable[8]
    0xC0, // 10dBm 0xC0, 7dBm 0xC8, 5dBm 0x84, 0dBm 0x60, -10dBm 0x34, -15dBm 0x1D, -20dBm 0x0E, -30dBm 0x12
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

const uint8_t subghz_device_cc1101_preset_2fsk_dev12khz_async_regs[] = {

    /* GPIO GD0 */
    CC1101_IOCFG0,
    0x0D, // GD0 as async serial data output/input

    /* Frequency Synthesizer Control */
    CC1101_FSCTRL1,
    0x06, // IF = (26*10^6) / (2^10) * 0x06 = 152343.75Hz

    /* Packet engine */
    CC1101_PKTCTRL0,
    0x32, // Async, continious, no whitening
    CC1101_PKTCTRL1,
    0x04,

    // // Modem Configuration
    CC1101_MDMCFG0,
    0x00,
    CC1101_MDMCFG1,
    0x02,
    CC1101_MDMCFG2,
    0x04, // Format 2-FSK/FM, No preamble/sync, Disable (current optimized)
    CC1101_MDMCFG3,
    0x83, // Data rate is 4.79794 kBaud
    CC1101_MDMCFG4,
    0x67, //Rx BW filter is 270.833333 kHz
    CC1101_DEVIATN,
    0x30, //Deviation ~12 kHz

    /* Main Radio Control State Machine */
    CC1101_MCSM0,
    0x18, // Autocalibrate on idle-to-rx/tx, PO_TIMEOUT is 64 cycles(149-155us)

    /* Frequency Offset Compensation Configuration */
    CC1101_FOCCFG,
    0x16, // no frequency offset compensation, POST_K same as PRE_K, PRE_K is 4K, GATE is off

    /* Automatic Gain Control */
    CC1101_AGCCTRL0,
    0x91, //10 - Medium hysteresis, medium asymmetric dead zone, medium gain ; 01 - 16 samples agc; 00 - Normal AGC, 01 - 8dB boundary
    CC1101_AGCCTRL1,
    0x00, // 0; 0 - LNA 2 gain is decreased to minimum before decreasing LNA gain; 00 - Relative carrier sense threshold disabled; 0000 - RSSI to MAIN_TARGET
    CC1101_AGCCTRL2,
    0x07, // 00 - DVGA all; 000 - MAX LNA+LNA2; 111 - MAIN_TARGET 42 dB

    /* Wake on radio and timeouts control */
    CC1101_WORCTRL,
    0xFB, // WOR_RES is 2^15 periods (0.91 - 0.94 s) 16.5 - 17.2 hours

    /* Frontend configuration */
    CC1101_FREND0,
    0x10, // Adjusts current TX LO buffer
    CC1101_FREND1,
    0x56,

    /* End load reg */
    0,
    0,

    // 2fsk_async_patable[8]
    0xC0, // 10dBm 0xC0, 7dBm 0xC8, 5dBm 0x84, 0dBm 0x60, -10dBm 0x34, -15dBm 0x1D, -20dBm 0x0E, -30dBm 0x12
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

const uint8_t subghz_device_cc1101_preset_2fsk_dev47_6khz_async_regs[] = {
    /* GPIO GD0 */
    CC1101_IOCFG0,   0x0D,  // GD0 async serial

    /* Frequency Synthesizer Control */
    CC1101_FSCTRL1,  0x06,  // IF

    /* Packet engine */
    CC1101_PKTCTRL0, 0x32,  // Async, continuous, no whitening
    CC1101_PKTCTRL1, 0x04,

    /* Modem Configuration */
    CC1101_MDMCFG0,  0x00,
    CC1101_MDMCFG1,  0x02,
    CC1101_MDMCFG2,  0x04,  // 2-FSK, no preamble/sync
    CC1101_MDMCFG3,  0x83,  // ~4.8 kBaud
    CC1101_MDMCFG4,  0x67,  // Rx BW ~270kHz
    CC1101_DEVIATN,  0x47,  // Dev ~47.6kHz

    /* Main Radio Control State Machine */
    CC1101_MCSM0,    0x18,  // autocal on idle->rx/tx

    /* Frequency Offset Compensation Configuration */
    CC1101_FOCCFG,   0x16,

    /* AGC */
    CC1101_AGCCTRL0, 0x91,
    CC1101_AGCCTRL1, 0x00,
    CC1101_AGCCTRL2, 0x07,

    /* WOR */
    CC1101_WORCTRL,  0xFB,

    /* Frontend */
    CC1101_FREND0,   0x10,
    CC1101_FREND1,   0x56,

    /* End marker */
    0, 0,

    /* PATABLE (8 bytes) */
    0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
