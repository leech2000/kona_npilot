from enum import IntFlag

from cereal import car
from panda.python import uds
from selfdrive.car import dbc_dict
from selfdrive.car.fw_query_definitions import FwQueryConfig, p16, Request

Ecu = car.CarParams.Ecu

class CarControllerParams:

  ACCEL_MAX = 2.0
  ACCEL_MIN = -3.5

  def __init__(self, CP):
    self.STEER_MAX = 380
    self.STEER_DELTA_UP = 3

    if CP.carFingerprint in [CAR.KONA_HEV, CAR.KONA_EV]:
      self.STEER_DELTA_DOWN = 7
      self.STEER_MAX = 380
    else:
      self.STEER_DELTA_DOWN = 6

    self.STEER_DRIVER_ALLOWANCE = 50
    self.STEER_DRIVER_MULTIPLIER = 2
    self.STEER_DRIVER_FACTOR = 1
    self.STEER_THRESHOLD = 150

    if CP.carFingerprint in CANFD_CAR:
      self.STEER_MAX = 270
      self.STEER_DRIVER_ALLOWANCE = 250
      self.STEER_DRIVER_MULTIPLIER = 2
      self.STEER_THRESHOLD = 250
      self.STEER_DELTA_UP = 2
      self.STEER_DELTA_DOWN = 3

class HyundaiFlags(IntFlag):
  CANFD_HDA2 = 1
  CANFD_ALT_BUTTONS = 2

class CAR:
  # genesis
  GENESIS = "HYUNDAI GENESIS 2015-2016"
  GENESIS_G70 = "GENESIS G70 2018"
  GENESIS_G80 = "GENESIS G80 2017"
  GENESIS_EQ900 = "GENESIS EQ900 2017"
  GENESIS_EQ900_L = "GENESIS EQ900 LIMOUSINE"
  GENESIS_G90 = "GENESIS G90 2019"
  # hyundai
  ELANTRA = "HYUNDAI ELANTRA 2017"
  ELANTRA_2021 = "HYUNDAI ELANTRA 2021"
  ELANTRA_HEV_2021 = "HYUNDAI ELANTRA HYBRID 2021"
  ELANTRA_GT_I30 = "HYUNDAI I30 N LINE 2019 & GT 2018 DCT"
  SONATA = "HYUNDAI SONATA 2020"
  SONATA_HEV = "HYUNDAI SONATA HYBRID 2020"
  SONATA21_HEV = "HYUNDAI SONATA HYBRID 2021"
  SONATA19_HEV = "HYUNDAI SONATA HYBRID 2019"
  SONATA19 = "HYUNDAI SONATA 2019"
  SONATA_LF_TURBO = "HYUNDAI SONATA LF TURBO"
  KONA = "HYUNDAI KONA 2020"
  KONA_EV = "HYUNDAI KONA ELECTRIC 2019"
  KONA_HEV = "HYUNDAI KONA HYBRID 2019"
  IONIQ = "HYUNDAI IONIQ HYBRID 2017-2019"
  IONIQ_EV_LTD = "HYUNDAI IONIQ ELECTRIC LIMITED 2019"
  IONIQ_EV_2020 = "HYUNDAI IONIQ ELECTRIC 2020"
  IONIQ_PHEV = "HYUNDAI IONIQ PHEV 2020"
  SANTA_FE = "HYUNDAI SANTA FE 2019"
  SANTA_FE_2022 = "HYUNDAI SANTA FE 2022"
  SANTA_FE_HEV_2022 = "HYUNDAI SANTA FE HYBRID 2022"
  PALISADE = "HYUNDAI PALISADE 2020"
  VELOSTER = "HYUNDAI VELOSTER 2019"
  GRANDEUR_IG = "HYUNDAI GRANDEUR IG 2017"
  GRANDEUR_IG_HEV = "HYUNDAI GRANDEUR IG HEV 2019"
  GRANDEUR_IG_FL = "HYUNDAI GRANDEUR IG FL 2020"
  GRANDEUR_IG_FL_HEV = "HYUNDAI GRANDEUR IG FL HEV 2020"
  TUCSON_TL_SCC  = "HYUNDAI TUCSON TL SCC 2017"
  IONIQ_5 = "HYUNDAI IONIQ 5 2022"
  TUCSON_HYBRID_4TH_GEN = "HYUNDAI TUCSON HYBRID 4TH GEN"

  # kia
  FORTE = "KIA FORTE E 2018 & GT 2021"
  K5 = "KIA K5 2019 & 2016"
  K5_2021 = "KIA K5 2021"
  K5_HEV = "KIA K5 HYBRID 2017 & SPORTS 2019"
  K5_HEV_2022 = "KIA K5 HYBRID 2022"
  SPORTAGE = "KIA SPORTAGE S 2020"  
  SORENTO = "KIA SORENTO GT LINE 2018"
  STINGER = "KIA STINGER GT2 2018"
  NIRO_EV = "KIA NIRO EV 2020 PLATINUM"
  NIRO_HEV = "KIA NIRO HEV 2018"
  NIRO_HEV_2021 = "KIA NIRO HEV 2021"
  CEED = "KIA CEED 2019"
  SELTOS = "KIA SELTOS 2021"
  MOHAVE = "KIA MOHAVE 2019"
  K7 = "KIA K7 2016-2019"
  K7_HEV = "KIA K7 HEV 2016-2019"
  K9 = "KIA K9 2016-2019"
  EV6 = "KIA EV6 2022"

class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4


FINGERPRINTS = {
  # genesis
  CAR.GENESIS: [{}],
  CAR.GENESIS_G70: [{}],
  CAR.GENESIS_G80: [{}],
  CAR.GENESIS_EQ900: [{}],
  CAR.GENESIS_EQ900_L: [{}],
  CAR.GENESIS_G90: [{}],
  # hyundai
  CAR.ELANTRA: [{}],
  CAR.ELANTRA_GT_I30: [{}],
  CAR.SONATA: [{}],
  CAR.SONATA_HEV: [{}],
  CAR.SONATA19: [{}],
  CAR.SONATA19_HEV: [{}],
  CAR.SONATA_LF_TURBO: [{}],
  CAR.KONA: [{}],
  CAR.KONA_EV: [{}],
  CAR.KONA_HEV: [{
    68: 8, 127: 8, 304: 8, 320: 8, 339: 8, 352: 8, 356: 4, 544: 8, 546: 8, 547: 8, 548: 8, 549: 8, 576: 8, 593: 8, 688: 5, 832: 8, 881: 8, 882: 8, 897: 8, 902: 8, 903: 8, 905: 8, 909: 8, 916: 8, 1040: 8, 1042: 8, 1056: 8, 1057: 8, 1078: 4, 1136: 6, 1138: 4, 1151: 6, 1155: 8, 1157: 4, 1164: 8, 1168: 7, 1173: 8, 1183: 8, 1186: 2, 1191: 2, 1193: 8, 1225: 8, 1265: 4, 1280: 1, 1287: 4, 1290: 8, 1291: 8, 1292: 8, 1294: 8, 1312: 8, 1322: 8, 1342: 6, 1345: 8, 1348: 8, 1355: 8, 1363: 8, 1369: 8, 1378: 8, 1379: 8, 1407: 8, 1419: 8, 1427: 6, 1429: 8, 1430: 8, 1448: 8, 1456: 4, 1470: 8, 1476: 8, 1535: 8, 1988: 8, 1996: 8, 2000: 8, 2004: 8, 2008: 8, 2012: 8
  }],
  CAR.IONIQ: [{}],
  CAR.IONIQ_EV_LTD: [{}],
  CAR.IONIQ_EV_2020: [{}],
  CAR.SANTA_FE: [{}],
  CAR.PALISADE: [{}],
  CAR.VELOSTER: [{}],
  CAR.GRANDEUR_IG: [{}],
  CAR.GRANDEUR_IG_HEV: [{}],
  CAR.GRANDEUR_IG_FL: [{}],
  CAR.GRANDEUR_IG_FL_HEV: [{}],
  CAR.TUCSON_TL_SCC: [{}],

  # kia
  CAR.FORTE: [{}],
  CAR.K5: [{}],
  CAR.K5_HEV: [{}],
  CAR.K5_HEV_2022: [{}],
  CAR.K5_2021: [{}],
  CAR.SPORTAGE: [{}],
  CAR.SORENTO: [{}],
  CAR.STINGER: [{}],
  CAR.NIRO_EV: [{}],
  CAR.NIRO_HEV: [{}],
  CAR.CEED: [{}],
  CAR.SELTOS: [{}],
  CAR.MOHAVE: [{}],
  CAR.K7: [{}],
  CAR.K7_HEV: [{}],
  CAR.K9: [{}],
}

HYUNDAI_VERSION_REQUEST_LONG = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf100)  # Long description
HYUNDAI_VERSION_REQUEST_MULTI = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_SOFTWARE_IDENTIFICATION) + \
  p16(0xf100)
HYUNDAI_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_MULTI],
      [HYUNDAI_VERSION_RESPONSE],
    ),
  ],
)

FW_VERSIONS = {
  CAR.EV6: {
    (Ecu.abs, 0x7d1, None): [
      b'\xf1\x00CV  IEB \x03 101!\x10\x18 58520-CV100',
      b'\xf1\x8758520CV100\xf1\x00CV  IEB \x02 101!\x10\x18 58520-CV100',
    ],
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00CV1 MDPS R 1.00 1.04 57700-CV000 1B30',
    ],
    (Ecu.fwdRadar, 0x7d0, None): [
      b'\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         ',
      b'\xf1\x8799110CV000\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         ',
    ],
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00CV1 MFC  AT USA LHD 1.00 1.05 99210-CV000 211027',
      b'\xf1\x00CV1 MFC  AT EUR LHD 1.00 1.05 99210-CV000 211027',
    ],
  },
  CAR.IONIQ_5: {
    (Ecu.abs, 0x7d1, None): [
      b'\xf1\x00NE1 IEB \x07 106!\x11) 58520-GI010',
      b'\xf1\x8758520GI010\xf1\x00NE1 IEB \x07 106!\x11) 58520-GI010',
    ],
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00NE  MDPS R 1.00 1.06 57700GI000  4NEDR106',
      b'\xf1\x8757700GI000 \xf1\x00NE  MDPS R 1.00 1.06 57700GI000  4NEDR106',
    ],
    (Ecu.fwdRadar, 0x7d0, None): [
      b'\xf1\x00NE1_ RDR -----      1.00 1.00 99110-GI000         ',
      b'\xf1\x8799110GI000\xf1\x00NE1_ RDR -----      1.00 1.00 99110-GI000         ',
    ],
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00NE1 MFC  AT USA LHD 1.00 1.02 99211-GI010 211206',
    ],
  },
  CAR.TUCSON_HYBRID_4TH_GEN: {
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00NX4 FR_CMR AT USA LHD 1.00 1.00 99211-N9240 14Q',
    ],
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00NX4 MDPS C 1.00 1.01 56300-P0100 2228',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x87391312MND0',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'\xf1\x00PSBG2441  G19_Rev\x00\x00\x00SNX4T16XXHS01NS2lS\xdfa',
      b'\xf1\x8795441-3D220\x00\xf1\x81G19_Rev\x00\x00\x00\xf1\x00PSBG2441  G19_Rev\x00\x00\x00SNX4T16XXHS01NS2lS\xdfa',
    ],
  },
}

CHECKSUM = {
  "crc8": [CAR.SANTA_FE, CAR.SONATA, CAR.PALISADE, CAR.SONATA_HEV, CAR.SONATA21_HEV, CAR.SELTOS, CAR.ELANTRA_2021,
           CAR.ELANTRA_HEV_2021, CAR.SANTA_FE_HEV_2022, CAR.K5_2021, CAR.K5_HEV_2022],
  "6B": [CAR.SORENTO, CAR.GENESIS, CAR.SANTA_FE_2022],
}

FEATURES = {
  # Use Cluster for Gear Selection, rather than Transmission
  "use_cluster_gears": {CAR.ELANTRA, CAR.KONA, CAR.ELANTRA_GT_I30, CAR.K7, CAR.GRANDEUR_IG, CAR.GRANDEUR_IG_FL},

  # Use TCU Message for Gear Selection
  "use_tcu_gears": {CAR.K5, CAR.SONATA19, CAR.VELOSTER, CAR.SONATA_LF_TURBO, CAR.TUCSON_TL_SCC},

  # Use E_GEAR Message for Gear Selection
  "use_elect_gears": {CAR.K5_HEV, CAR.K5_HEV_2022, CAR.IONIQ_EV_LTD, CAR.KONA_EV, CAR.KONA_HEV, CAR.SONATA_HEV, CAR.SONATA21_HEV, CAR.SONATA21_HEV, CAR.NIRO_EV, CAR.K7_HEV,
                      CAR.GRANDEUR_IG_HEV, CAR.GRANDEUR_IG_FL_HEV, CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.ELANTRA_HEV_2021,
                      CAR.NIRO_HEV, CAR.NIRO_HEV_2021, CAR.SANTA_FE_HEV_2022},

  # send LFA MFA message for new HKG models
  "send_lfa_mfa": {CAR.KONA_HEV},

  # these cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
  "use_fca": {CAR.KONA_HEV},

  "has_scc13": {CAR.PALISADE, CAR.NIRO_HEV, CAR.NIRO_HEV_2021, CAR.K9, CAR.GENESIS_G90, CAR.K5_2021, CAR.MOHAVE},
  "has_scc14": {CAR.PALISADE, CAR.NIRO_HEV, CAR.NIRO_HEV_2021, CAR.K9, CAR.GENESIS_G90, CAR.K5_2021, CAR.MOHAVE},

  "send_mdps12": {CAR.K9, CAR.GENESIS_G90},
}

HYBRID_CAR = {CAR.KONA_HEV}

EV_CAR = {CAR.IONIQ_EV_LTD, CAR.IONIQ_EV_2020, CAR.KONA_EV, CAR.NIRO_EV}

EV_HYBRID_CAR = EV_CAR | HYBRID_CAR

CANFD_CAR = {CAR.EV6, CAR.IONIQ_5, CAR.TUCSON_HYBRID_4TH_GEN}

DBC = {
  # genesis
  CAR.GENESIS: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_G70: dbc_dict('hyundai_kia_generic', None),  
  CAR.GENESIS_G80: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_EQ900: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_EQ900_L: dbc_dict('hyundai_kia_generic', None),
  CAR.GENESIS_G90: dbc_dict('hyundai_kia_generic', None),
  # hyundai
  CAR.ELANTRA: dbc_dict('hyundai_kia_generic', None),
  CAR.ELANTRA_2021: dbc_dict('hyundai_kia_generic', None),
  CAR.ELANTRA_HEV_2021: dbc_dict('hyundai_kia_generic', None),
  CAR.ELANTRA_GT_I30: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA21_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA19: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA19_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.SONATA_LF_TURBO: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA_EV: dbc_dict('hyundai_kia_generic', None),
  CAR.KONA_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ_EV_LTD: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ_PHEV: dbc_dict('hyundai_kia_generic', None),
  CAR.IONIQ_EV_2020: dbc_dict('hyundai_kia_generic', None),
  CAR.SANTA_FE: dbc_dict('hyundai_kia_generic', None),
  CAR.SANTA_FE_2022: dbc_dict('hyundai_kia_generic', None),
  CAR.SANTA_FE_HEV_2022: dbc_dict('hyundai_kia_generic', None),
  CAR.PALISADE: dbc_dict('hyundai_kia_generic', None),
  CAR.VELOSTER: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG_FL: dbc_dict('hyundai_kia_generic', None),
  CAR.GRANDEUR_IG_FL_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.TUCSON_TL_SCC: dbc_dict('hyundai_kia_generic', None),
  # kia
  CAR.FORTE: dbc_dict('hyundai_kia_generic', None),
  CAR.K5: dbc_dict('hyundai_kia_generic', None),
  CAR.K5_2021: dbc_dict('hyundai_kia_generic', None),
  CAR.K5_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.K5_HEV_2022: dbc_dict('hyundai_kia_generic', None),
  CAR.SPORTAGE: dbc_dict('hyundai_kia_generic', None),  
  CAR.SORENTO: dbc_dict('hyundai_kia_generic', None),
  CAR.STINGER: dbc_dict('hyundai_kia_generic', None),  
  CAR.NIRO_EV: dbc_dict('hyundai_kia_generic', None),
  CAR.NIRO_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.NIRO_HEV_2021: dbc_dict('hyundai_kia_generic', None),
  CAR.CEED: dbc_dict('hyundai_kia_generic', None),
  CAR.SELTOS: dbc_dict('hyundai_kia_generic', None),
  CAR.MOHAVE: dbc_dict('hyundai_kia_generic', None),
  CAR.K7: dbc_dict('hyundai_kia_generic', None),
  CAR.K7_HEV: dbc_dict('hyundai_kia_generic', None),
  CAR.K9: dbc_dict('hyundai_kia_generic', None),

  # CAN FD
  CAR.EV6: dbc_dict('hyundai_canfd', None),
  CAR.IONIQ_5: dbc_dict('hyundai_canfd', None),
  CAR.TUCSON_HYBRID_4TH_GEN: dbc_dict('hyundai_canfd', None),
}



def main():
  for member, value in vars(CAR).items():
    if not member.startswith("_"):
      print(value)


if __name__ == "__main__":
  main()
