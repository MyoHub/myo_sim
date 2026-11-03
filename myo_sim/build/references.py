"""Cited table values for the rigid-map and passive-fit scripts.

Transcribed published numbers only. Papers and OpenSim models stay out of the repo.

Rajagopal et al. 2016 IEEE Trans Biomed Eng 63:2068-2079, doi:10.1109/TBME.2016.2586891.
SimTK full_body is Apache-2.0; Rajagopal2016.osim is not redistributed.

Holzbaur et al. 2005 Ann Biomed Eng 33:829-840, doi:10.1007/s10439-005-3320-7;
Saul et al. 2015 Comput Methods Biomech Biomed Engin 18:1445-1458, doi:10.1080/10255842.2014.916698.
SimTK MoBL-ARMS is BSD-3 with a non-commercial restriction; MOBL_ARMS_41.osim is not redistributed.

Silder et al. 2007 J Biomech 40:2628-2635, doi:10.1016/j.jbiomech.2006.12.017.
Table 4 means only; subject recordings are not stored.
"""

from __future__ import annotations

RAJAGOPAL2016_LEG = {  # Rajagopal et al. 2016; l0/lts m, alpha rad
    "citation": "Rajagopal A, Dembia CL, DeMers MS, Delp DD, Hicks JL, Delp SL (2016) IEEE Trans Biomed Eng 63:2068-2079",
    "doi": "10.1109/TBME.2016.2586891",
    "muscles": {
        "addbrev_r": {"l0": 0.1031, "lts": 0.03545, "alpha": 0.114781, "status": "adopted"},
        "addlong_r": {"l0": 0.1082, "lts": 0.131799, "alpha": 0.13777, "status": "adopted"},
        "addmagDist_r": {"l0": 0.1772, "lts": 0.087381, "alpha": 0.194705, "status": "adopted"},
        "addmagIsch_r": {"l0": 0.1562, "lts": 0.216343, "alpha": 0.168044, "status": "kept"},
        "addmagMid_r": {"l0": 0.1377, "lts": 0.046628, "alpha": 0.207308, "status": "adopted"},
        "addmagProx_r": {"l0": 0.1056, "lts": 0.040324, "alpha": 0.311483, "status": "kept"},
        "bflh_r": {"l0": 0.0976, "lts": 0.325227, "alpha": 0.175918, "status": "adopted"},
        "bfsh_r": {"l0": 0.1103, "lts": 0.105817, "alpha": 0.264223, "status": "adopted"},
        "edl_r": {"l0": 0.0693, "lts": 0.368875, "alpha": 0.218258, "status": "kept"},
        "ehl_r": {"l0": 0.0748, "lts": 0.326795, "alpha": 0.197268, "status": "kept"},
        "fdl_r": {"l0": 0.0446, "lts": 0.378773, "alpha": 0.224839, "status": "kept"},
        "fhl_r": {"l0": 0.0527, "lts": 0.35434, "alpha": 0.258026, "status": "adopted"},
        "gaslat_r": {"l0": 0.0588, "lts": 0.37607, "alpha": 0.210227, "status": "adopted"},
        "gasmed_r": {"l0": 0.051, "lts": 0.398716, "alpha": 0.165682, "status": "kept"},
        "glmax1_r": {"l0": 0.147, "lts": 0.048841, "alpha": 0.354012, "status": "kept"},
        "glmax2_r": {"l0": 0.157, "lts": 0.067887, "alpha": 0.367382, "status": "adopted"},
        "glmax3_r": {"l0": 0.167, "lts": 0.069717, "alpha": 0.382416, "status": "kept"},
        "glmed1_r": {"l0": 0.073, "lts": 0.055817, "alpha": 0.316556, "status": "adopted"},
        "glmed2_r": {"l0": 0.073, "lts": 0.065249, "alpha": 0.316556, "status": "adopted"},
        "glmed3_r": {"l0": 0.073, "lts": 0.045233, "alpha": 0.316556, "status": "adopted"},
        "glmin1_r": {"l0": 0.068, "lts": 0.016138, "alpha": 0.174533, "status": "adopted"},
        "glmin2_r": {"l0": 0.056, "lts": 0.026099, "alpha": 0.0, "status": "adopted"},
        "glmin3_r": {"l0": 0.038, "lts": 0.050873, "alpha": 0.017453, "status": "kept"},
        "grac_r": {"l0": 0.2278, "lts": 0.172014, "alpha": 0.172002, "status": "adopted"},
        "iliacus_r": {"l0": 0.1066, "lts": 0.096121, "alpha": 0.279914, "status": "kept"},
        "perbrev_r": {"l0": 0.0454, "lts": 0.147528, "alpha": 0.205236, "status": "adopted"},
        "perlong_r": {"l0": 0.0508, "lts": 0.332221, "alpha": 0.247952, "status": "adopted"},
        "piri_r": {"l0": 0.026, "lts": 0.114906, "alpha": 0.174533, "status": "adopted"},
        "psoas_r": {"l0": 0.1169, "lts": 0.099543, "alpha": 0.215525, "status": "adopted"},
        "recfem_r": {"l0": 0.0759, "lts": 0.448494, "alpha": 0.217015, "status": "kept"},
        "sart_r": {"l0": 0.403, "lts": 0.124, "alpha": 0.026135, "status": "adopted"},
        "semimem_r": {"l0": 0.069, "lts": 0.347586, "alpha": 0.254561, "status": "adopted"},
        "semiten_r": {"l0": 0.193, "lts": 0.247199, "alpha": 0.241295, "status": "adopted"},
        "soleus_r": {"l0": 0.044, "lts": 0.276756, "alpha": 0.381429, "status": "kept"},
        "tfl_r": {"l0": 0.095, "lts": 0.449497, "alpha": 0.05236, "status": "kept"},
        "tibant_r": {"l0": 0.0683, "lts": 0.240461, "alpha": 0.195183, "status": "kept"},
        "tibpost_r": {"l0": 0.0378, "lts": 0.28078, "alpha": 0.226489, "status": "kept"},
        "vasint_r": {"l0": 0.0993, "lts": 0.202211, "alpha": 0.0631, "status": "adopted"},
        "vaslat_r": {"l0": 0.0994, "lts": 0.220601, "alpha": 0.252867, "status": "kept"},
        "vasmed_r": {"l0": 0.0968, "lts": 0.199904, "alpha": 0.422223, "status": "adopted"},
    },
}

MOBL_ARM = {  # Holzbaur 2005 / Saul 2015; l0/lts m, alpha rad
    "citation": "Holzbaur et al. 2005 Ann Biomed Eng 33:829-840; Saul et al. 2015 Comput Methods Biomech Biomed Engin 18:1445-1458",
    "doi": "10.1007/s10439-005-3320-7; 10.1080/10255842.2014.916698",
    "muscles": {
        "DELT1": {"l0": 0.0976, "lts": 0.093, "alpha": 0.383972, "status": "kept"},
        "DELT2": {"l0": 0.1078, "lts": 0.1095, "alpha": 0.261799, "status": "adopted"},
        "DELT3": {"l0": 0.1367, "lts": 0.038, "alpha": 0.314159, "status": "adopted"},
        "SUPSP": {"l0": 0.0682, "lts": 0.0395, "alpha": 0.122173, "status": "kept"},
        "INFSP": {"l0": 0.0755, "lts": 0.0308, "alpha": 0.331613, "status": "kept"},
        "SUBSC": {"l0": 0.0873, "lts": 0.033, "alpha": 0.349066, "status": "adopted"},
        "TMIN": {"l0": 0.0741, "lts": 0.0713, "alpha": 0.418879, "status": "kept"},
        "TMAJ": {"l0": 0.1624, "lts": 0.02, "alpha": 0.279253, "status": "adopted"},
        "PECM1": {"l0": 0.1442, "lts": 0.0028, "alpha": 0.296706, "status": "kept"},
        "PECM2": {"l0": 0.1385, "lts": 0.089, "alpha": 0.453786, "status": "kept"},
        "PECM3": {"l0": 0.1385, "lts": 0.132, "alpha": 0.436332, "status": "adopted"},
        "LAT1": {"l0": 0.254, "lts": 0.12, "alpha": 0.436332, "status": "adopted"},
        "LAT2": {"l0": 0.2324, "lts": 0.1765, "alpha": 0.331613, "status": "adopted"},
        "LAT3": {"l0": 0.2789, "lts": 0.1403, "alpha": 0.366519, "status": "adopted"},
        "CORB": {"l0": 0.0932, "lts": 0.097, "alpha": 0.471239, "status": "kept"},
        "TRIlong": {"l0": 0.134, "lts": 0.143, "alpha": 0.20944, "status": "kept"},
        "TRIlat": {"l0": 0.1138, "lts": 0.098, "alpha": 0.15708, "status": "kept"},
        "TRImed": {"l0": 0.1138, "lts": 0.0908, "alpha": 0.15708, "status": "kept"},
        "ANC": {"l0": 0.027, "lts": 0.018, "alpha": 0.0, "status": "adopted"},
        "SUP": {"l0": 0.033, "lts": 0.028, "alpha": 0.0, "status": "kept"},
        "BIClong": {"l0": 0.1157, "lts": 0.2723, "alpha": 0.0, "status": "kept"},
        "BICshort": {"l0": 0.1321, "lts": 0.1923, "alpha": 0.0, "status": "kept"},
        "BRA": {"l0": 0.0858, "lts": 0.0535, "alpha": 0.0, "status": "adopted"},
        "BRD": {"l0": 0.1726, "lts": 0.133, "alpha": 0.0, "status": "kept"},
        "ECRL": {"l0": 0.081, "lts": 0.244, "alpha": 0.0, "status": "kept"},
        "ECRB": {"l0": 0.0585, "lts": 0.2223, "alpha": 0.15708, "status": "kept"},
        "ECU": {"l0": 0.0622, "lts": 0.2285, "alpha": 0.069813, "status": "kept"},
        "FCR": {"l0": 0.0628, "lts": 0.244, "alpha": 0.05236, "status": "kept"},
        "FCU": {"l0": 0.0509, "lts": 0.265, "alpha": 0.20944, "status": "kept"},
        "PL": {"l0": 0.0638, "lts": 0.2694, "alpha": 0.069813, "status": "kept"},
        "PT": {"l0": 0.0492, "lts": 0.098, "alpha": 0.174533, "status": "kept"},
        "PQ": {"l0": 0.0282, "lts": 0.005, "alpha": 0.174533, "status": "kept"},
        "FDSL": {"l0": 0.0515, "lts": 0.3386, "alpha": 0.087266, "status": "kept"},
        "FDSR": {"l0": 0.0736, "lts": 0.328, "alpha": 0.069813, "status": "kept"},
        "FDSM": {"l0": 0.0749, "lts": 0.295, "alpha": 0.122173, "status": "kept"},
        "FDSI": {"l0": 0.0835, "lts": 0.275, "alpha": 0.10472, "status": "kept"},
        "FDPL": {"l0": 0.0749, "lts": 0.2819, "alpha": 0.139626, "status": "kept"},
        "FDPR": {"l0": 0.0798, "lts": 0.2915, "alpha": 0.122173, "status": "kept"},
        "FDPM": {"l0": 0.0835, "lts": 0.303, "alpha": 0.10472, "status": "kept"},
        "FDPI": {"l0": 0.0749, "lts": 0.3015, "alpha": 0.122173, "status": "kept"},
        "EDCL": {"l0": 0.065, "lts": 0.335, "alpha": 0.034907, "status": "kept"},
        "EDCR": {"l0": 0.0626, "lts": 0.365, "alpha": 0.05236, "status": "kept"},
        "EDCM": {"l0": 0.0724, "lts": 0.365, "alpha": 0.05236, "status": "kept"},
        "EDCI": {"l0": 0.07, "lts": 0.365, "alpha": 0.05236, "status": "kept"},
        "EDM": {"l0": 0.0675, "lts": 0.335, "alpha": 0.05236, "status": "kept"},
        "EIP": {"l0": 0.0589, "lts": 0.21, "alpha": 0.10472, "status": "kept"},
        "EPL": {"l0": 0.054, "lts": 0.2305, "alpha": 0.10472, "status": "kept"},
        "EPB": {"l0": 0.0675, "lts": 0.1155, "alpha": 0.122173, "status": "kept"},
        "FPL": {"l0": 0.0552, "lts": 0.197, "alpha": 0.122173, "status": "kept"},
        "APL": {"l0": 0.0713, "lts": 0.1295, "alpha": 0.139626, "status": "kept"},
    },
}

SILDER2007 = {  # Table 4 means; subject recordings are not stored
    "citation": "Silder A, Whittington B, Heiderscheit B, Thelen DG (2007) Identification of passive elastic joint moment-angle relationships in the lower extremity. J Biomech 40:2628-2635",
    "doi": "10.1016/j.jbiomech.2006.12.017",
    "params": {
        "RF": {"b_prox": 3.1, "b_dist": 1.9, "a_deg": 24.4},
        "HAM": {"b_prox": 5.1, "b_dist": 3.9, "a_deg": 30.8},
        "GAS": {"b_prox": 4.5, "b_dist": 4.7, "a_deg": 26.0},
        "HF": {"b": 5.1, "a_deg": 19.5},
        "HE": {"b": 2.0, "a_deg": 27.3},
        "KF": {"b": 5.8, "a_deg": 12.8},
        "KE": {"b": 3.5, "a_deg": 101.9},
        "PF": {"b": 4.9, "a_deg": -3.4},
    },
    "validation": {"rms_Nm": {"hip": 11.0, "knee": 8.38, "ankle": 2.06}},
}
