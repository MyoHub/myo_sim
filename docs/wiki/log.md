# Wiki Maintenance Log

Append-only. Add entries at the top (most recent first). Do not edit past entries.

---

## 2026-11-04 — CMC4/5 springs to active cupping, ORL to Ueba's shares, EDC2's wrist sphere
Changed: `scripts/fit_cmc_joints.py`, `scripts/fit_extensor_mechanism.py`, `build/data/cmc_joints.json`, `extensor_mechanism.json`, `extrinsic_muscles.json`, `passive_joint_moments.json`, `muscle_passive.json`, `myoarm_r_chain.xml`, `myoarm_r_tendon.xml`, `myoarm_r_muscle.xml`, `tests/test_extensor_mechanism.py`, `hand-cmc-joints.md`, `extensor-mechanism.md`, `hand-literature-comparison.md`, `CHANGELOG.md`
Why: Active cupping overshot its measured maxima, the ORL carried nothing above PIP 0 against Ueba's shares, and EDC2 ran clear of the dorsal wrist; springs, ligament and sphere are refitted and every passive fit rerun.

## 2026-11-04 — One specific tension for the wrist and hand muscles
Changed: `scripts/fit_specific_tension.py`, `build/data/specific_tension.json`, `hand_intrinsic_muscles.json`, `extrinsic_muscles.json`, `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json`, `myoarm_r_muscle.xml`, `myoarm_r_tendon.xml`, `myoarm_r_chain.xml`, `scripts/validate_hand_function.py`, `scripts/generate_extrinsic_muscles.py`, `tests/test_specific_tension.py`, `tests/test_hand_grip.py`, `hand-function-validation.md`, `hand-literature-comparison.md`, `passive-moments-and-thenar.md`, `CHANGELOG.md`
Why: The hand's muscles sat at specific tensions from 86 to 248 N/cm2 with no measurement behind them; one value calibrated to measured strength replaces them, with its conflict against the physiological range stated.

## 2026-11-04 — Grip and fist diagnosis; share test on whole fingers, settled
Changed: `scripts/validate_hand_function.py`, `scripts/hand_literature_comparison.py`, `tests/test_hand_grip.py`, `hand_function.json`, `hand_literature.json`, `hand-function-validation.md`, `hand-literature-comparison.md`, `CHANGELOG.md`
Why: Kong and Amis measure whole fingers and Kong's subjects held for 4 s, while the test read two phalanges at 0.15 s; the diagnosis also shows MCP collateral strands help only that transient, so they are documented and not adopted.

## 2026-11-04 — Contact margin between adjacent digits
Changed: `myo_sim/build/data/hand_contact_pairs.json`, `scripts/generate_hand_contacts.py`, `myo_sim/models/contacts/myohand_self_contacts.xml`, `hand-contacts-and-pads.md`, `hand_literature.json`, `hand-literature-comparison.md`, `CHANGELOG.md`
Why: Rigid capsules of adjacent fingers overlap by up to 8.7 mm at the flexion limits, so the middle finger wedged between its neighbours in a fist; contact now starts where the bones would meet.

## 2026-11-04 — Hand contact pairs from the active fist
Changed: `myo_sim/build/data/hand_contact_pairs.json`, `scripts/generate_hand_contacts.py` (group label per `selection.always` entry), `myo_sim/models/contacts/myohand_self_contacts.xml`, `hand-contacts-and-pads.md`, `arm/README_hand.md`, `hand_literature.json`, `hand-literature-comparison.md`, `CHANGELOG.md`
Why: The static scan cannot see postures past a soft joint limit, so the ring fingertip entered its own metacarpal skin in the active fist and held the middle finger back; five pairs close this.

## 2026-11-04 — Wrap geoms without mass
Changed: `myo_sim/models/arm/assets/myoarm_r_assets.xml`, `myoarm_r_chain.xml` (lunate inertial), `tests/test_mass_convention.py`, `tests/test_chest_ownership.py`, `hand_literature.json`, `hand-literature-comparison.md`, `CHANGELOG.md`
Why: Arm wrap geoms carried mass through the default density, so resizing a wrap changed the lunate and the body total; they are massless now and the lunate keeps its `dev` mass, which returns `myofullbody` to 84.00 kg.

## 2026-11-03 — Generated-block tests across MuJoCo versions; open checks
Changed: `myo_sim/build/hand_data.py` (`blocks_match`), `tests/test_passive_thenar.py`, `test_extensor_mechanism.py`, `test_extrinsic_muscles.py`, `README.md`, `myo_sim/models/head/README.md`, `arm/README_hand.md`, `hand-literature-comparison.md`, `CHANGELOG.md`
Why: On MuJoCo 3.17 generated blocks differed from the scripts' output in last digits (at most 2e-10), the wrap test's negative control has nothing to detect since mujoco-warp 3.16, and a spring refit differed by 4e-8 N m between versions. These tests now hold on both versions; a real difference still fails them (`test_block_comparison_rejects_a_real_difference`). Open checks state where the free speeds and grip shares stand.

## 2026-11-03 — Deep hip muscles part of the leg model
Changed: `myo_sim/models/leg/assets/myolegs_chain.xml`, `myolegs_tendon.xml`, `myolegs_muscle.xml` (generated blocks), `myo_sim/build/data/deep_hip_muscles.json`, `LICENSE-Gait2392.txt`, `scripts/generate_deep_hip_muscles.py`, `scripts/hand_literature_comparison.py`, `tests/test_deep_hip_muscles.py`, `test_sims.py`, `test_head_build.py`, `test_hand_literature.py`, `myo_sim/models/leg/README.md`, `CHANGELOG.md`
Why: The Rajagopal legs lack pectineus, quadratus femoris and the gemelli. The opt-in branch (`hip_synergists=True`, Python-built) becomes XML without a flag under the model policy (#140), because the static tests pass and the step cost is within 2 %. The length maps follow #123's rule over the full hip range.

## 2026-11-03 — The muscular neck merged into the rebuilt myofullbody
Changed: `tests/test_head_build.py`, `test_mass_convention.py`, `test_chest_ownership.py` (merged counts and total mass), `scripts/hand_literature_comparison.py` (`neck_strength`, head-and-neck share), `tests/test_hand_literature.py`, `hand_literature.json`, `hand-literature-comparison.md`, `docs/validation/neck/README.md`, `CHANGELOG.md`
Why: The neck branch brings Vittorio's neck into the full body only; its counts and the 84.0 kg total predate the rebuilt hand. Strength is re-measured here on the masses and inertias now in the model, which the earlier recording could not show.

## 2026-11-03 — Merge #123's rigid-tendon maps into the hand rebuild
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml` (ten arm maps on the `_r` actuators), `myo_sim/models/leg/assets/myolegs_muscle.xml`, `myo_sim/build/rigid_conversion.py` (arm region's actuator suffix), `tests/test_arm_rigid_conversion.py`, `hand_literature.json`, `hand-literature-comparison.md`, `CHANGELOG.md`
Why: #123's follow-up maps had to come into the rebuild; the arm table keeps MoBL-ARMS' names and the fit now targets the suffixed actuators. No forearm or hand muscle changes, and the refit confirms it. Cited values of the rigid maps stay in `build/references.py`, as #123 has them; the hand's tables stay in `build/data/`.

## 2026-11-03 — ECU through the groove of the ulnar head; forearm rotation axis checked
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (ECU groove block on the ulna; ECU via point, wrap sphere and side sites removed from the radius), `myoarm_r_tendon.xml` (ECU path, springs), `myoarm_r_muscle.xml` (refitted passive parameters), `myo_sim/build/data/extrinsic_muscles.json` (`ecu`), `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json` (grip cache), `hand_literature.json`, `scripts/generate_extrinsic_muscles.py` (`--fit-ecu`, forearm landmarks), `scripts/hand_literature_comparison.py` (forearm rotation), `tests/test_extrinsic_muscles.py`, `test_hand_literature.py`, `test_muscle_passive.py` (left/right settling compared at solver tolerance 1e-12), `docs/wiki/hand-extrinsic-muscles.md`, `hand-literature-comparison.md`, `hand-contacts-and-pads.md`, `index.md`, `repository-map.md`, `testing-guide.md`, `CHANGELOG.md`
Why: The ECU crossed the wrist from a via point on the radius 20 mm from the rotation axis, so its arms ignored forearm rotation; it now leaves the groove of the ulnar head, as measured by Horii 1993 and Loren 1996, while the rotation axis already passes through the radial and ulnar heads and stays.
## 2026-11-03 — Numerical stability: vmax and thumb CMC armature
Changed: `myo_sim/models/arm/assets/myoarm_r_assets.xml` (`vmax` of `myoarm_forearm_muscle`), `myoarm_r_chain.xml` (CMC armature), `myo_sim/build/data/muscle_passive.json` (`force_velocity`), `passive_joint_moments.json` (`armature`), `hand_function.json` (grip optimum, optimiser starts), `hand_literature.json`, `myo_sim/build/stability.py`, `tests/test_numerical_stability.py`, `scripts/fit_muscle_passive.py`, `scripts/fit_passive_moments.py`, `scripts/hand_literature_comparison.py`, `docs/wiki/hand-literature-comparison.md`, `hand-function-validation.md`, `passive-moments-and-thenar.md`, `testing-guide.md`, `CHANGELOG.md`
Why: The wrist and digit joints alternated velocity sign at 2 ms from activation 0.3 because the muscles' force-velocity damping, explicit under Euler, exceeded the stability limit once fibre lengths were anatomical with `vmax` at MuJoCo's default; `vmax` 10 and a CMC armature of 2e-4 kg m2 remove it and leave the static results unchanged.

## 2026-10-30 — Grip digit shares: measured target
Changed: `myo_sim/build/data/hand_function.json` (digit-share target and band, sources), `hand_literature.json` (grip-share literature rows, sources), `tests/test_hand_grip.py` (mean +- 1 SD band), `docs/wiki/hand-function-validation.md`, `hand-literature-comparison.md`
Why: The grip test compared the digit shares with the ARMS model's own shares; it now uses measured shares during maximal power grip (Kong et al. 2011, 100 % MVC, mean +- 1 SD), with Amis 1987, Ohtsuki 1981 and Ejeskar & Ortengren 1981 as context. The model is outside the measured band for every finger, so the test keeps failing.

## 2026-10-30 — Forearm wrap side sites outside their geoms
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (side sites and four wrap geoms), `myoarm_r_muscle.xml`, `myoarm_r_tendon.xml` (refitted parameters, springs), `myo_sim/build/data/extrinsic_muscles.json` (`wrap_sides` replaces `edc2_wrap_side`), `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json` (grip cache), `hand_literature.json` (ECRL, ECRB and FCR rows; step wrap_sides), `scripts/generate_extrinsic_muscles.py` (`wrap_sides`), `scripts/hand_literature_comparison.py`, `tests/test_extrinsic_muscles.py` (all wrap sides; CPU vs mujoco-warp with a negative control), `tests/test_hand_literature.py`, `docs/wiki/hand-extrinsic-muscles.md`, `hand-literature-comparison.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: Side sites inside their geoms make MuJoCo apply the inside wrap, which MuJoCo and mujoco-warp solve differently; as for EDC2, the forearm wraps now wrap from outside, with ECRB and FCR moved into Horii 1993's bands. ECRL stays open.

## 2026-10-30 — S2c: AdPt insertion and APL CMC routing after Barry et al. 2018
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (AdPt sites, APL sites, chain block `thumb trapezium`), `myoarm_r_tendon.xml` (APL path, springs), `myoarm_r_muscle.xml` (refitted parameters), `myo_sim/build/data/LICENSE-Barry2018.txt` (new), `extrinsic_muscles.json` (`thumb_extrinsics`, sources), `hand_intrinsic_muscles.json` (AdPt), `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json` (grip cache), `hand_literature.json` (FPL and APL rows, step s2c), `scripts/generate_extrinsic_muscles.py` (thumb block; commented-out sites ignored when rewriting positions), `scripts/hand_literature_comparison.py`, `tests/test_hand_literature.py`, `tests/test_passive_thenar.py` (class default fpmax for rows without their own), `docs/wiki/hand-extrinsic-muscles.md`, `passive-moments-and-thenar.md`, `hand-literature-comparison.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: Barry et al. 2018's model shares MyoHand's bone frames; its AdPt insertion and APL routing bring the thumb's moment arms closer to Smutz et al. 1998 with the lateral pinch in its band. Its FPL routing would too but makes the pinch infeasible, so it is documented and not used.

## 2026-10-30 — Junctura node ties of JT3 and JT5
Changed: `scripts/generate_extrinsic_muscles.py` (helpers from `EDC{d}-P4_r`, no reference sites, link rest length over two postures), `myo_sim/models/arm/assets/myoarm_r_chain.xml`, `myoarm_r_tendon.xml`, `myoarm_r_muscle.xml`, `myo_sim/build/data/extrinsic_muscles.json`, `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json` (grip cache; fingerprint gains the EDC4-EDC5 junctura stiffness), `hand_literature.json`, source access notes in `hand_intrinsic_muscles.json`, `muscle_passive.json` and `passive_joint_moments.json` (wording), `tests/test_extrinsic_muscles.py` (node vs tendon excursion with a negative control), `tests/test_hand_grip.py` (fingerprint), `tests/test_muscle_passive.py` (left-right comparison at rest and solver tolerance 1e-12), `docs/wiki/hand-extrinsic-muscles.md`, `hand-literature-comparison.md` (also two open checks from Barry et al. 2018's distributed model), `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: The JT3 and JT5 tie helpers started near the MC head and missed the MCP wrap the EDC takes from P4, so those nodes moved 6-11 mm more than the tendon material; starting the helpers at P4 makes every node follow its tendon.

## 2026-10-30 — Shared terminal tendon of the lateral bands (step time)
Changed: `myo_sim/models/arm/assets/myoarm_r_tendon.xml` (EDC2-5, EIP, EDM and the junctura tie helpers), `scripts/fit_extensor_mechanism.py` (`pulley_fractions`), `tests/test_extensor_mechanism.py`, `myo_sim/build/data/hand_literature.json`, `docs/wiki/extensor-mechanism.md`, `hand-literature-comparison.md` (also an open check on the JT3/JT5 node ties), `CHANGELOG.md`
Why: A profile of the CPU step found the two band branches of each extensor computing the same terminal tendon and DIP wrap twice; one shared branch with both fractions gives the same lengths and moment arms at 1-2 % less step time.

## 2026-10-30 — EDC2 wrist wrap: side site dorsal of its sphere
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (`EDCI_torus_site_EDC2_side_r`), `myoarm_r_muscle.xml` (refitted passive parameters), `myoarm_r_tendon.xml` (junctura links, springs), `myo_sim/build/data/extrinsic_muscles.json` (`edc2_wrap_side`, junctura fit), `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json` (grip cache), `hand_literature.json`, `scripts/generate_extrinsic_muscles.py` (side site from the table; the table is written on every run), `tests/test_extrinsic_muscles.py` (side site, CPU vs mujoco-warp EDC2 length with a negative control), `docs/wiki/hand-extrinsic-muscles.md`, `hand-literature-comparison.md`, `passive-moments-and-thenar.md`, `index.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: EDC2's side site sat at the centre of its wrap sphere, so MuJoCo applied the inside wrap, which MuJoCo and mujoco-warp solve differently; off-centre but inside changes nothing, so the site moved out of the sphere on its dorsal side.

## 2026-10-29 — Release checks of the hand rebuild: open questions and accepted design choices
Changed: `docs/wiki/hand-literature-comparison.md` (section "Open checks before release", new subsection "Accepted design choices")
Why: Before the release of the revised hand, the adopted values that the literature questions are listed with their evidence (one specific tension for intrinsics and extrinsics, hood fractions, extensor share, ORL slack, IP coupling, FDP links), and the numerical settings kept as final (lumbrical node armature and damping, lumbrical slack rule, JT34 at its stiffness bound) are recorded as such.

## 2026-10-29 — Juncturae tendinum, FCU and FDS5 (hand rebuild S7)
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (node blocks JT3-5, FCU hamate block, FCU sites), `myoarm_r_tendon.xml` (junctura ties and links, FCU path), `myoarm_r_assets.xml` (tie equalities), `myoarm_r_muscle.xml` (FCU length range, FDS5 force, refitted passive parameters), `myo_sim/build/data/extrinsic_muscles.json` (new), `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json` (grip cache), `hand_literature.json`, `scripts/generate_extrinsic_muscles.py` (new), `scripts/hand_literature_comparison.py`, `tests/test_extrinsic_muscles.py` (new), joint counts in `test_sims.py`, `test_fragment_registry.py`, `test_myohand_equivalence.py`, `test_lumbrical_nodes.py`, `test_muscle_passive.py` (contact settling exception moved to MCP4 at 60 deg wrist flexion), `docs/wiki/hand-extrinsic-muscles.md` (new), `hand-literature-comparison.md`, `index.md`, `repository-map.md`, `testing-guide.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: The extensor tendons of the ulnar fingers were independent although the juncturae couple them; FCU ran past the pisiform without the hamate hook and with flexion and ulnar arms outside Horii's bands; FDS5 carried almost half of FDS4's force against a quarter of its PCSA.

## 2026-10-29 — Lumbrical junction nodes (hand rebuild S6)
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (node bodies and slides, labelled blocks), `myoarm_r_assets.xml` (tendon-equality ties), `myoarm_r_tendon.xml` (helper tendons, springs), `myoarm_r_muscle.xml` (length ranges, slack lengths, fpmax), `myo_sim/build/data/hand_intrinsic_muscles.json` (`lumbrical_nodes`), `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json` (grip cache), `hand_literature.json`, `myo_sim/build/hand_data.py` (`place_tendon_nodes`, `fold_node_forces`), `myo_sim/build/utils.py` (slide axes and equality tendon references mirrored), `scripts/generate_hand_intrinsic_muscles.py` (`--place-nodes`, node blocks, node-aware path fit), `scripts/fit_extensor_mechanism.py`, `fit_muscle_passive.py`, `fit_passive_moments.py`, `validate_hand_function.py`, `hand_literature_comparison.py`, `tests/test_lumbrical_nodes.py` (new), joint counts in `test_sims.py`, `test_fragment_registry.py`, `test_myohand_equivalence.py`, `test_passive_thenar.py`, `test_hand_literature.py`, `docs/wiki/passive-moments-and-thenar.md`, `hand-literature-comparison.md`, `repository-map.md`, `testing-guide.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: The lumbricals arose from fixed points in the palm, so they did not lengthen as the fingers flexed (Ranney & Wells 1988: +31 mm into the fist); each now arises from a node tied to its FDP tendon, as in the hand.

## 2026-10-29 — Interossei and lumbricals by identity (hand rebuild S5)
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (new sites, old bundle sites removed), `myoarm_r_tendon.xml`, `myoarm_r_muscle.xml` (ten labelled blocks, springs, extrinsic length ranges and passive scales), `myo_sim/build/data/hand_intrinsic_muscles.json`, `extensor_mechanism.json` (intrinsic rows moved out, lumbrical fractions fitted), `cmc_joints.json` (interosseous site moves now start points), `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json`, `hand_literature.json`, `myo_sim/build/hand.py`, `scripts/generate_hand_intrinsic_muscles.py` (branched rows), `scripts/fit_extensor_mechanism.py` (junction branches shared, lumbrical fit), `scripts/validate_hand_function.py`, `scripts/hand_literature_comparison.py`, `tests/test_interossei_lumbricals.py` (new), `test_extensor_mechanism.py`, `test_passive_thenar.py`, `test_cmc_joints.py`, `test_hand_literature.py`, `docs/wiki/extensor-mechanism.md`, `passive-moments-and-thenar.md`, `hand-cmc-joints.md`, `hand-function-validation.md`, `hand-literature-comparison.md`, `testing-guide.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: The interossei and lumbricals were lumped by finger side, which inverted the anatomy of four of them, and their forces followed no rule (lumbricals up to 1225 N/cm2); each is now one anatomical muscle with Jacobson's architecture, its insertion into bone and hood after Salsbury 1937, and paths fitted to An 1979/1983.

## 2026-10-29 — Finger extensor mechanism, option (a) (hand rebuild EM-a)
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (junction sites and band wraps; obsolete extensor and band sites removed), `myoarm_r_tendon.xml` (branched tendons, ORL ligaments, springs), `myoarm_r_muscle.xml` (length ranges, slack lengths, fpmax), `myo_sim/build/data/extensor_mechanism.json` (new), `muscle_passive.json`, `passive_joint_moments.json`, `hand_function.json` (grip cache), `hand_literature.json`, `scripts/fit_extensor_mechanism.py` (new), `scripts/fit_passive_moments.py` (ligaments in the baseline), `scripts/hand_literature_comparison.py`, `tests/test_extensor_mechanism.py` (new), `test_passive_thenar.py`, `test_muscle_passive.py`, `docs/wiki/extensor-mechanism.md` (new), `hand-literature-comparison.md`, `hand-contacts-and-pads.md`, `passive-muscles-tenodesis.md`, `index.md`, `repository-map.md`, `testing-guide.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: Extensor force reached the DIP only through the long extensors and no intrinsic crossed the DIP; the extensors now split into central slip and lateral bands with measured fractions, the band-inserting intrinsics extend PIP and DIP, and an oblique retinacular ligament couples the two passively.

## 2026-10-29 — Left-right settling with touching fingers (hand rebuild)
Changed: `tests/test_hand_contacts.py` (pair order and geom roles), `tests/test_muscle_passive.py` (0.2 deg for MCP3/MCP4 at 60 deg wrist extension), `docs/wiki/hand-contacts-and-pads.md`
Why: Since S4 the left and right MCP3/MCP4 settled 0.10-0.15 deg apart with both wrists extended; the contact pairs are mirrored exactly, and the difference comes from the contact solver's tolerance with static friction, not from the model.

## 2026-10-29 — Thumb CMC axes after Hollister 1992 tested, not adopted (hand rebuild S2b)
Changed: `docs/wiki/hand-literature-comparison.md` (sections "Tested and not adopted" and "Open checks before release"), `docs/wiki/passive-moments-and-thenar.md` (pointer)
Why: Hollister's non-intersecting thumb CMC axes were tried to make Smutz's moment-arm trends reachable; they do not (the thumb-intrinsic match got worse), so the model is unchanged and the evidence is recorded, together with the adopted values that need a check before the release.

## 2026-10-29 — Hypothenar muscles (hand rebuild S4)
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (hypothenar sites; UI_UB5 sites removed), `myoarm_r_tendon.xml`, `myoarm_r_muscle.xml` (ADM labelled block, FDM and ODM in the main block, springs), `myo_sim/build/data/hand_intrinsic_muscles.json`, `passive_joint_moments.json`, `hand_function.json`, `hand_literature.json`, `myo_sim/build/hand.py`, `scripts/generate_hand_intrinsic_muscles.py` (little-finger boxes, passive fit over named rows), `scripts/validate_hand_function.py`, `scripts/hand_literature_comparison.py`, `tests/test_passive_thenar.py`, `test_cmc_joints.py`, `test_hand_literature.py`, `test_muscle_passive.py` (docstring), actuator counts in `test_sims.py`, `test_passive_torso_build.py`, `test_fragment_registry.py`, `test_myohand_equivalence.py`, `docs/wiki/passive-moments-and-thenar.md`, `hand-cmc-joints.md`, `hand-function-validation.md`, `hand-literature-comparison.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: The little finger had no hypothenar muscles: UI_UB5 stood in for the abductor with an interosseous-like path, and FDM and ODM were missing, so the palm could not cup actively once CMC4/5 existed.

## 2026-10-29 — Fourth and fifth carpometacarpal joints (hand rebuild S3)
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (joints, moved sites), `myoarm_r_assets.xml` (coupling block), `myoarm_r_tendon.xml` (CMC springs), `myo_sim/build/data/cmc_joints.json` (new), `passive_joint_moments.json`, `hand_function.json`, `hand_literature.json`, `scripts/fit_cmc_joints.py` (new), `scripts/hand_literature_comparison.py`, `tests/test_cmc_joints.py` (new), `test_hand_grip.py`, `test_hand_literature.py`, joint counts in `test_sims.py`, `test_fragment_registry.py`, `test_myohand_equivalence.py`, `docs/wiki/hand-cmc-joints.md` (new), `hand-function-validation.md`, `hand-literature-comparison.md`, `index.md`, `repository-map.md`, `testing-guide.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: MC4 and MC5 were fixed to the capitate, so the palm could not cup and the digit-5 muscles' share of the grip was low; the coupled hinges follow measured axes, arcs and the active ratio, and the sites on the wrong bone are moved.

## 2026-10-29 — Thumb and first-dorsal-interosseous heads (hand rebuild S2)
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (intrinsic sites), `myoarm_r_tendon.xml`, `myoarm_r_muscle.xml` (main and labelled generated blocks, springs), `myo_sim/build/data/hand_intrinsic_muscles.json` (was `thenar_muscles.json`), `LICENSE-Mirakhorlo2016.txt`, `passive_joint_moments.json`, `hand_function.json`, `hand_literature.json`, `myo_sim/build/hand_data.py` (labelled blocks), `myo_sim/build/hand.py`, `scripts/generate_hand_intrinsic_muscles.py` (was `generate_thenar_muscles.py`), `scripts/validate_hand_function.py`, `scripts/hand_literature_comparison.py`, `tests/test_passive_thenar.py`, `test_hand_pinch.py`, `test_hand_grip.py`, `test_hand_literature.py`, actuator counts in `test_sims.py`, `test_passive_torso_build.py`, `test_fragment_registry.py`, `test_myohand_equivalence.py`, `docs/wiki/passive-moments-and-thenar.md`, `hand-function-validation.md`, `hand-literature-comparison.md`, `index.md`, `repository-map.md`, `testing-guide.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: The thumb adductor, the short thumb flexor and the first dorsal interosseous were single bundles with paths that did not match published moment arms; each head now has its own actuator, and the paths of OP, APB, FPB and DI1 are fitted to Smutz 1998 and An 1983 from one table and script.

## 2026-10-29 — PIP flexion range (hand rebuild S1b)
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (PIP ranges, palm pads), `myoarm_r_tendon.xml` (EDM path, PIP springs), `myo_sim/build/data/passive_joint_moments.json`, `hand_pads.json`, `hand_function.json` (grip cache), `hand_literature.json`, `scripts/fit_passive_moments.py` (writes `range`), `scripts/hand_literature_comparison.py`, `tests/test_passive_thenar.py`, `tests/test_hand_literature.py`, `docs/wiki/passive-moments-and-thenar.md`, `hand-contacts-and-pads.md`, `hand-function-validation.md`, `hand-literature-comparison.md`, `myo_sim/models/arm/README.md`, `README_hand.md`, `CHANGELOG.md`
Why: The PIP joints stopped at 90 deg, below the measured maximum passive flexion (115 +- 7 deg, Leijnse 2010), so in the fist the fingertips stayed far from the bones and the fitted palm pad took 19 mm. With the measured limit the palm pad is 8 mm and the grip is back inside its band.

## 2026-10-29 — Hand pads and self-contacts (hand rebuild S1)
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml` (pads, distal capsules), `myo_sim/models/contacts/myohand_self_contacts.xml` (new), removed `myo_sim/models/contacts/myohand_contacts.xml`, `myo_sim/build/compose.py`, `myo_sim/build/data/hand_pads.json`, `hand_contact_pairs.json`, `hand_function.json` (grip cache), `hand_literature.json`, `scripts/fit_hand_pads.py`, `scripts/generate_hand_contacts.py`, `scripts/validate_hand_function.py` (scenes remove the model's pairs), `scripts/hand_literature_comparison.py`, `tests/test_hand_contacts.py`, `tests/test_hand_grip.py`, `tests/test_hand_literature.py`, `tests/test_passive_thenar.py`, `tests/test_contact_paths.py`, `docs/wiki/hand-contacts-and-pads.md`, `hand-function-validation.md`, `hand-literature-comparison.md`, `build-and-composition.md`, `repository-map.md`, `testing-guide.md`, `index.md`, `CHANGELOG.md`
Why: The fingertip pads sat on the nail side and the palm had no soft tissue, so fingertips could not touch the palm and pulp pinch had no contact; finger contacts existed only in the hand-only models. The pads are now palmar and fitted (Dallard 2018; the fist), and one generated set of self-contacts goes into every model with a hand.

## 2026-10-29 — Side suffix on every right-arm name
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml`, `myoarm_r_tendon.xml`, `myoarm_r_muscle.xml`, `myo_sim/build/utils.py` (mirror rule `_r_tendon` -> `_l_tendon`), `myo_sim/build/hand.py`, `myo_sim/build/data/muscle_passive.json`, `passive_joint_moments.json`, `thenar_muscles.json`, `hand_function.json`, `scripts/fit_muscle_passive.py`, `scripts/generate_thenar_muscles.py`, `scripts/validate_hand_function.py`, `scripts/compare_model_numerics.py`, `tests/test_naming.py` and the tests that name arm elements, `docs/wiki/model-authoring.md`, `build-and-composition.md`, `passive-moments-and-thenar.md`, `hand-function-validation.md`, `testing-guide.md`, `myo_sim/models/arm/README.md`, `CHANGELOG.md`
Why: The right arm was the only part whose joints, actuators, tendons, geoms and cameras lacked `_r` (MoBL legacy), so the same muscle was `OP` in `myoarm_r` and `OP_r` in `myohand_r`. Every right-arm name now carries the side, muscle tendons follow their actuator (`OP_r_tendon`) as in the legs and torso, and a script proves the compiled numerics are unchanged.

## 2026-10-29 — Hand models by generic pruning; literature comparison page
Changed: `myo_sim/build/hand.py`, `myo_sim/build/compose.py`, `myo_sim/models/contacts/myohand_contacts.xml`, `tests/test_hand_pruning.py`, `scripts/hand_literature_comparison.py`, `myo_sim/build/data/hand_literature.json`, `tests/test_hand_literature.py`, `docs/wiki/hand-literature-comparison.md`, `docs/wiki/build-and-composition.md`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/testing-guide.md`, `docs/wiki/index.md`, `myo_sim/build/README.md`, `CHANGELOG.md`
Why: The hand models stay removal-only derivations of the canonical arm, but the pruning now handles every reference MjSpec cannot follow (tendon paths, equalities, sensors, pairs) generically instead of case by case, and a test proves every surviving arm element is in the hand unchanged. The new page records the hand against published data after each step of the hand rebuild.

## 2026-11-01 — Drop build/data
Changed: removed `myo_sim/build/data/` (JSON and LICENSE-*.txt); citations live in `myo_sim/build/references.py`
Why: no data directory; the committed product is the XML and cited table constants sit next to the fit code.

## 2026-11-01 — No JSON in build/data
Changed: removed `build/data/*.json`; cited table values moved to `myo_sim/build/references.py`; `build/data/` keeps `LICENSE-*.txt` only
Why: model policy: raw datasets stay out of the repo; the committed product is the XML; cited numbers are code constants, not data files.

## 2026-11-01 — Provenance JSON is cited tables only
Changed: slimmed `build/data/*.json` to citation/DOI/licence plus table rows; moved fit grids out of JSON; dropped `scripts/debug_leg_physiology.py`; `--write` no longer dumps fit logs back into JSON
Why: model policy (MyoHub/myo_sim#140): `build/data/<topic>.json` holds one row per element, not optimizer output, posture grids or analysis scripts.

## 2026-11-01 — Rigid-map follow-up follows model-policy provenance
Changed: removed `tests/data/*.npz`; fit CLIs under `scripts/fit_*.py`; `LICENSE-*.txt` next to `build/data`; CHANGELOG; `docs/wiki/model-authoring.md`
Why: model policy (MyoHub/myo_sim#140): raw datasets and papers are never committed; derived table values live in `build/data/<topic>.json` with citation, DOI and licence; fit scripts are `scripts/fit_*.py` and run only at development time.

## 2026-10-31 — Arm rigid-tendon maps from MoBL-ARMS, checked along measured shoulder motion
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `myo_sim/build/rigid_conversion.py` (regions), `myo_sim/build/data/mobl_arm_muscles.json`, `tests/test_arm_rigid_conversion.py`, `tests/data/seth2019_shoulder_motion.npz`, `myo_sim/models/arm/README.md`
Why: Along the measured shoulder motions of Seth et al. (2019), LAT1, LAT3 and PECM3 operated outside 0.5-1.5 L0 in 100 %, 64 % and 56 % of frames. Ten muscles now use maps converted directly from MoBL-ARMS (0-5 % of frames outside, closer to the Seth model's fibre lengths); the selection rules are recorded in the data file.

## 2026-10-31 — Leg passive force calibrated to measured passive joint moments
Changed: `myo_sim/models/leg/assets/myolegs_muscle.xml`, `myo_sim/build/leg_passive.py`, `myo_sim/build/data/leg_passive.json`, `tests/test_leg_passive.py`, `tests/test_leg_rigid_conversion.py`, `tests/data/rajagopal2016_leg_opensim.npz`, `tests/debug_leg_physiology.py`, `myo_sim/models/leg/README.md`
Why: With the converted maps the hamstrings still gave 19-41 Nm of passive knee moment where people show 6-30 Nm, and MuJoCo's default passive scale did not reflect measured passive moments anywhere. Tendon slack length and fpmax per muscle are fitted to Silder et al. (2007), keeping the gastrocnemius at Narici et al. (1996) fascicle lengths and each muscle at half its active force or more at Anderson et al. (2007) strength angles; hip/knee/ankle RMS 19.8/11.2/2.6 -> 11.0/8.4/2.1 N m. Torques in the physiology script now use effective moment arms (knee translations and patella resolved), which changes the knee numbers reported on this date.

## 2026-10-31 — Leg rigid-tendon muscle maps derived directly from the Rajagopal reference
Changed: `myo_sim/models/leg/assets/myolegs_muscle.xml`, `myo_sim/build/rigid_conversion.py`, `myo_sim/build/data/rajagopal2016_leg_muscles.json`, `tests/test_leg_rigid_conversion.py`, `tests/debug_leg_physiology.py`, `docs/wiki/model-authoring.md`, `docs/wiki/testing-guide.md`, `docs/wiki/repository-map.md`, `myo_sim/models/leg/README.md`
Why: The 2026-08-02 leg refit kept anatomical optimal fibre lengths but anchored them to a gait-range `lengthrange`; outside it MuJoCo extrapolates, and semimembranosus reached ln ~2.9 with the hip flexed (161 Nm passive knee moment). Fitting MuJoCo's linear map to the reference's rigid-tendon fibre length over the full joint range reproduces OpenSim's fibre lengths (median RMSE 0.02, max 0.10, over 400 postures) and was adopted for the 24 muscles where it raises the isometric force error by at most 2 % of Fmax; checked against cadaver fibre lengths, ultrasound fascicle lengths and dynamometry.

## 2026-10-28 — BIClong no longer wraps its elbow sphere
Changed: myo_sim/models/arm/assets/myoarm_r_tendon.xml, tests/test_model_stability.py, tests/test_tendon_path_continuity.py, tests/data/myofullbody_biclong_elbow_graze.json, docs/wiki/testing-guide.md, myo_sim/models/arm/README.md, CHANGELOG.md
Why: The wrap only acted past the elbow extension limit, where it gave NaN BIClong lengths (MuJoCo < 3.16) and 15-20 mm length jumps in random-activation rollouts of myofullbody.

## 2026-10-28 — CPU vs mujoco-warp tendon length parity test
Changed: tests/test_warp_parity.py, docs/wiki/testing-guide.md, myo_sim/models/arm/README.md
Why: EDC2 and the other tendons with a sidesite inside their wrap geom differed between CPU MuJoCo and mujoco-warp < 3.16 by up to about 3 mm (float32 inside-wrap fallback, google-deepmind/mujoco_warp#1713), so every hand and arm tendon is now checked on both backends.

## 2026-10-29 — Merge dev; mass convention test covers the neck chain
Changed: tests/test_mass_convention.py, tests/test_head_build.py, myo_sim/build/mass.py, CHANGELOG.md, docs/wiki/testing-guide.md
Why: dev brought #138's mass test, which measured the head share on the `head` body and expected 84.3 kg. With the neck in `myofullbody`, the share is taken over the `myohead_base` subtree and the total is 84.0 kg; part models are checked for #138's rigid head. The neck mass check moves from `test_head_build.py` into this test, and `audit_mass` uses the neck chain when present.

## 2026-10-28 — Head and neck mass, cervical inertias and neck armature
Changed: myo_sim/models/head/assets/myohead_chain.xml, myo_sim/models/head/assets/myohead_assets.xml, myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, tests/test_head_build.py, tests/test_chest_ownership.py, myo_sim/models/head/README.md, docs/validation/neck/README.md, docs/wiki/testing-guide.md, CHANGELOG.md
Why: the neck raised body mass by 5.3 kg while the torso mass still contained the head. Head and neck now carry the de Leva share (6.94 %) and the torso drops to 14.897 kg as in #138; cervical inertias were 0.04 kg·m² placeholders, about 100 times too large, and come from a cylinder-slice approximation; with them the neck rang at 2 ms, so its joints use the torso's armature of 0.01. The commented-out rigid-head joints are removed.

## 2026-10-28 — Make the muscular neck part of myofullbody
Changed: myo_sim/build/compose.py, tests/test_head_build.py, tests/test_build_registry.py, tests/test_sims.py, tests/test_chest_ownership.py, README.md, myo_sim/models/head/README.md, docs/validation/neck/README.md, docs/wiki/build-and-composition.md, scripts/render_composed_models.py, docs/images/models/myofullbody.png, CHANGELOG.md (new)
Why: Model policy (#140): realism improvements overwrite the reference model and there are no feature flags. The neck costs +14-17 % `mj_step` time on `myofullbody`, below the threshold for a separate variant, so the `include_neck` flag and `myofullbody_neck` are removed. Neck actuators are appended last to keep existing action indices. The part models, `myotorso`, `myotorso_arms` and `myotorso_arm_r` included, keep the rigid head until the simple-versus-full model strategy is decided.

## 2026-11-03 — Standing keyframe, 0.2.4
Changed: scripts/generate_standing_keyframes.py, myo_sim/build/keyframes.py, myo_sim/build/data/standing_keyframes.json, myo_sim/build/compose.py, myo_sim/__init__.py, tests/test_standing_keyframe.py, docs/wiki/standing-keyframe.md, docs/wiki/index.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md, docs/wiki/build-and-composition.md, CHANGELOG.md, pyproject.toml, uv.lock
Why: qpos0 of the free-standing models violates the knee couplings and overlaps the arms with the thorax, so they had no clean start pose; the keyframe adds one without changing any model parameter.

## 2026-11-03 — Composition fixes from #150
Changed: myo_sim/build/compose.py, myo_sim/build/utils.py, tests/test_build_registry.py, tests/test_contact_paths.py, tests/test_legs_abdomen_build.py, tests/test_mirror_symmetry.py, tests/test_equivalence.py, scripts/render_composed_models.py, README.md, CHANGELOG.md, docs/wiki/build-and-composition.md, docs/wiki/engineering-standards.md, docs/wiki/repository-map.md, docs/wiki/testing-guide.md
Why: the build_kwargs leak, the pairs kept by disable_finger_collisions, the abdomen variants' contacts and heading, the mirrored cameras, and docs and a test that still used the removed build_model().

## 2026-11-04 — Knee joint centre
Changed: myo_sim/models/leg/assets/myolegs_chain.xml, tests/test_knee_joint_centre.py, CHANGELOG.md, docs/wiki/testing-guide.md
Why: the knee's rotations were anchored at the tibia's origin; they now sit at Rajagopal's walker_knee joint frame, as in the source model.

## 2026-11-04 — LTpT_T12_l mirrored
Changed: myo_sim/models/torso/assets/myotorso_chain.xml, tests/test_torso_muscle_symmetry.py, myo_sim/models/torso/README.md, CHANGELOG.md
Why: the left T12 longissimus had the right muscle's path points, as in the source model; they now mirror the right ones.

## 2026-11-04 — Pelvis counted once
Changed: myo_sim/models/torso/assets/myotorso_chain.xml, tests/test_pelvis_mass_once.py, tests/test_mass_convention.py, tests/test_chest_ownership.py, docs/wiki/testing-guide.md, CHANGELOG.md
Why: the sacrum carried the lumbar model's pelvis mass while the leg chain brings its own pelvis, so the models with legs counted it twice.

## 2026-11-04 — Knee coupled ranges
Changed: myo_sim/models/leg/assets/myolegs_chain.xml, tests/test_knee_coupling_ranges.py, CHANGELOG.md, docs/wiki/testing-guide.md
Why: five coordinates the knee drives left their declared ranges along their own coupling polynomials.

## 2026-10-27 — Head and trunk mass convention
Changed: myo_sim/models/head/assets/myohead_rigid_chain.xml, myo_sim/models/torso/assets/myotorso_chain.xml, myo_sim/build/mass.py, tests/test_mass_convention.py, CHANGELOG.md
Why: the OpenSim torso mass already contained the head, leaving the head at about 2.5 % of body mass; head set to 6.94 % (de Leva) at unchanged total mass.

## 2026-10-28 — RI2 path, Kamper finger damping, pinch and tenodesis validation after McFarland et al. 2023
Changed: `myo_sim/models/arm/assets/myoarm_r_tendon.xml`, `myoarm_r_chain.xml`, `myo_sim/build/data/passive_joint_moments.json`, `scripts/fit_passive_moments.py`, `tests/test_passive_thenar.py`, `scripts/validate_hand_function.py`, `myo_sim/build/data/hand_function.json`, `myo_sim/build/hand_function.py`, `tests/test_hand_pinch.py`, `tests/test_hand_tenodesis.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/hand-function-validation.md`, `docs/wiki/index.md`, `docs/wiki/testing-guide.md`, `myo_sim/models/arm/README.md`, `myo_sim/models/arm/README_hand.md`
Why: RI2's path jumped by 7 mm because its insertion entered a wrap cylinder; finger damping was three to six times Kamper's measurement; the McFarland et al. 2023 validation found both. The pinch and tenodesis protocols and tests come with it (the script, table and loader are identical in Vittorio-Caggiano/myo_sim#2).

## 2026-10-27 — Thenar muscles and passive joint moments in the arm's own XML; springs fitted to the residual
Changed: `myo_sim/models/arm/assets/myoarm_r_tendon.xml`, `myoarm_r_muscle.xml` (generated blocks), removed `myoarm_r_thenar_*.xml` and `myoarm_r_passive*_tendon.xml`, `myo_sim/build/compose.py`, `myo_sim/build/utils.py`, `myo_sim/__init__.py`, `myo_sim/build/hand_data.py`, `scripts/fit_passive_moments.py`, `scripts/generate_thenar_muscles.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/build-and-composition.md`, `docs/wiki/repository-map.md`, `docs/wiki/testing-guide.md`
Why: One reference model per part, no flags: the thenar muscles and all measured and assumed passive rows are in the right-arm files (script-written blocks), and every spring is fitted to the source curve minus the model's own passive muscle moment, so nothing is counted twice; the `thenar_muscles` / `passive_moments` flags, the separate fragments and `subject_scale` (later PRs) are removed.
## 2026-10-28 — Release step for EDC3-5 and EDM; grip validation after McFarland et al. 2023
Changed: `scripts/fit_muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `tests/test_muscle_passive.py`, `scripts/validate_hand_function.py`, `myo_sim/build/data/hand_function.json`, `myo_sim/build/hand_function.py`, `tests/test_hand_grip.py`, `docs/wiki/passive-muscles-tenodesis.md`, `docs/wiki/hand-function-validation.md`, `docs/wiki/index.md`, `docs/wiki/testing-guide.md`, `myo_sim/models/arm/README.md`
Why: EDC3-5 and EDM sat at the fpmax floor, so the digits barely opened when the wrist flexed; only their fpmax is refit, above the smallest value that opens digits 3-5 in the McFarland et al. 2023 release trial, with the slack lengths unchanged. The grip protocol and test of that validation come with it (the script, table and loader are identical in MyoHub/myo_sim#139).

## 2026-10-27 — Fit fpmax with the slack lengths, under an active force-length constraint
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `scripts/fit_muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `tests/test_muscle_passive.py`, `docs/wiki/passive-muscles-tenodesis.md`, `docs/wiki/testing-guide.md`
Why: Lengthening the slack lengths alone left the finger flexors with almost no active force in a fist, so each extrinsic muscle now also gets its own passive-force scale (`fpmax`) and the fit keeps the flexors' active factor at least 0.5 in a full fist and the extensors' in full extension.

## 2026-10-27 — Fit the slack lengths of the other extrinsic finger, thumb and wrist muscles
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `scripts/fit_muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `tests/test_muscle_passive.py`, `docs/wiki/passive-muscles-tenodesis.md`, `docs/wiki/index.md`, `docs/wiki/testing-guide.md`
Why: On the anatomical parameters FDP4, FPL, FDP5, FDS3/FDP3 and ECU gave passive moments up to 25 times the measured totals (wrist, MCP/PIP/DIP 3-5, thumb MP/IP), so their slack lengths are now fitted per Binder-Markey to those totals with a one-sided loss (changes 0-10 %).

## 2026-10-27 — Fitted index extrinsic slack lengths written into the arm muscle XML
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `scripts/fit_muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `myo_sim/build/compose.py`, `myo_sim/__init__.py`, `tests/test_muscle_passive.py`, `docs/wiki/passive-muscles-tenodesis.md`, `docs/wiki/index.md`, `docs/wiki/testing-guide.md`
Why: Review: muscle parameters belong in the part's XML, not in a build step. The fitted `lengthrange` of FDS2/FDP2/EDC2/EIP is now in `myoarm_r_muscle.xml` (written by the fit script; the JSON stays as provenance), so tenodesis is in every model; the unreleased `passive_muscles` flag, `build/muscle_passive.py` and its `subject_scale` slack factor are removed.

## 2026-10-27 — Add opt-in passive muscle tension of the index extrinsic muscles (tenodesis)
Changed: `myo_sim/build/muscle_passive.py`, `myo_sim/build/data/muscle_passive.json`, `scripts/fit_muscle_passive.py`, `myo_sim/build/compose.py`, `myo_sim/__init__.py`, `tests/test_muscle_passive.py`, `docs/wiki/passive-muscles-tenodesis.md`
Why: With the anatomical slack lengths and rigid tendons, FDS2/FDP2/EDC2/EI give up to 327 N cm of passive index MCP moment against at most 30 N cm measured (Knutson et al. 2000); `passive_muscles=True` sets slack lengths fitted per Binder-Markey & Murray 2017 (RMSE 98.4 -> 3.0 N cm) and gives tenodesis on both hands. Off by default, so models and checkpoints are unchanged. The other digits and the thumb show the same excess but have no extrinsic passive data.

## 2026-08-02 — Widen force-length curve bounds for muscles whose refit operating range outgrew them
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`, `myo_sim/models/torso/assets/myotorso_muscle.xml`
Why: Restoring anatomical fiber length for 26 muscles (see the anatomical-parameter entries below) shifted their normalized operating range partly outside the shared default force-length curve bounds (`lmin`/`lmax`); widened bounds per muscle to contain the new range, respecting MuJoCo's `lmin < 1 < lmax` requirement.

## 2026-08-02 — Add inter-finger self-collision pairs to the hand
Changed: `myo_sim/models/contacts/myohand_contacts.xml`
Why: No finger-to-finger contact pairs existed at all (only forearm-torso and left/right-forearm pairs), so fingers could pass through each other; added adjacent-digit pairs at each phalanx level plus thumb-opposition pairs, verified on both single-hand and bilateral builds.

## 2026-08-02 — Widen wrist deviation and shoulder rotation ranges
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml`
Why: Real motion-capture data (throwing, catching, tennis) required wrist deviation and shoulder rotation angles beyond the shipped joint limits; widened both to cover the observed range plus a literature-consistent margin.

## 2026-08-02 — Correct lumbar axial rotation and wrist flexion joint ranges
Changed: `myo_sim/models/torso/assets/myotorso_chain.xml`, `myo_sim/models/arm/assets/myoarm_r_chain.xml`
Why: `L4_L5_AR` was ~18x wider than published per-segment lumbar ROM and wrist `flexion_r` was roughly half the typical clinical range; corrected both to match standard normative values.

## 2026-08-02 — Restore anatomical muscle parameters for hand extrinsics
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`
Why: All 24 flagged finger/wrist flexor-extensor muscles had never received individual calibration and were compiling against MuJoCo's raw default fiber-length range; fit each to its OpenSim/MoBL-ARMS anatomical optimal fiber length, tendon slack length, and peak force.

## 2026-08-02 — Restore anatomical muscle parameters for shoulder muscles
Changed: `myo_sim/models/arm/assets/myoarm_r_muscle.xml`
Why: 7 of 8 shoulder muscles flagged with inflated optimal fiber length were refit to their anatomical values; the 8th (`CORB`) was left unchanged after its excursion pattern matched the profile of muscles where refitting measurably hurts force accuracy.

## 2026-08-02 — Restore anatomical muscle parameters for the leg
Changed: `myo_sim/models/leg/assets/myolegs_muscle.xml`
Why: 6 of 10 flagged leg muscles were refit to anatomical optimal fiber length/tendon slack/peak force after remeasuring their physiological length range; the other 4 were deliberately left unchanged because refitting was cross-validated to increase force error against the OpenSim reference, not reduce it.

## 2026-08-02 — Restore anatomical muscle parameters for the torso
Changed: `myo_sim/models/torso/assets/myotorso_muscle.xml`
Why: 25 torso muscles had optimal fiber length inflated 5-30x versus the anatomical reference (a conversion artifact, not a real path-length problem); refit each to match, remeasuring the physiological joint range for the two muscles whose fit initially came out physically invalid.
## 2026-08-12 — Remove 12.87 kg of phantom mass from the `chest_r` wrap scaffold
Changed: `myo_sim/models/torso/assets/myotorso_assets.xml`, `myo_sim/models/torso/assets/myotorso_chain.xml`, `tests/test_chest_ownership.py`, `docs/wiki/testing-guide.md`
Why: `chest_r` contains only muscle-wrap ellipsoids and had no `<inertial>`, and the `myotorso_wrap` default class set no mass or density, so MuJoCo meshed its six r=0.08 m spheres at the default 1000 kg/m^3 — 6 x (4/3)pi(0.08)^3 x 1000 = 12.868 kg of parasitic mass welded into the torso. `build_model("myofullbody")` totalled 97.132 kg against the MuscleMimic reference's 84.270 kg, with `chest_r` the only body that differed; MuscleMimic sets its equivalent `thorax` body to 0.001 kg, and Winter's anthropometric segment fractions applied to the thigh, shank, foot and forearm masses all imply an 81-84 kg subject. Added `mass="0"` to the `myotorso_wrap` default class so no wrap geom can ever contribute mass again, plus an explicit `mass="0.001"` inertial on `chest_r` (required — with every geom massless and no inertial, MuJoCo rejects the body with "mass and inertia must be larger than mjMINVAL"). The only other `myotorso_wrap` geom, `pelvis_wrap` in `lumbar5`, sits in a body with its own explicit 1.824 kg inertial and is unaffected. Total is now 84.265 kg and `chest_r` is the sole body whose mass changed. The explicit inertial survives `spec.to_xml()` round-tripping, so the `explicitinertial` workaround in `myo_sim/build/elastic.py` is not needed here.

## 2026-10-27 — Thenar muscles and passive joint moments as arm XML fragments, on by default
Changed: `myo_sim/models/arm/assets/myoarm_r_chain.xml`, `myoarm_r_thenar_tendon.xml`, `myoarm_r_thenar_muscle.xml`, `myoarm_r_passive_tendon.xml`, `myoarm_r_passive_assumed_tendon.xml`, `myo_sim/build/compose.py`, `myo_sim/build/utils.py`, `myo_sim/build/hand.py`, `myo_sim/build/hand_data.py`, `scripts/fit_passive_moments.py`, `scripts/generate_thenar_fragments.py`, `docs/wiki/passive-moments-and-thenar.md`, `docs/wiki/model-authoring.md`, `docs/wiki/build-and-composition.md`, `docs/wiki/repository-map.md`, `docs/wiki/testing-guide.md`
Why: Review of the first version: the muscles and springs were added from Python instead of the part's XML files. Sites now live in the chain, tendons and actuators in fragments written by scripts from the JSON tables, merged and mirrored by compose.py; `thenar_muscles` / `passive_moments` default to on (+3 actuators per hand) and opt out to the previous model; MuJoCo floor raised to 3.7 for the polynomial tendon stiffness.
## 2026-10-04 — Align myohead muscle naming with other body parts
Changed: myo_sim/models/head/assets/, tests/test_head_build.py, myo_sim/models/head/README.md, docs/validation/neck/README.md, docs/wiki/repository-map.md
Why: Drop the `myohead_` actuator prefix, use `{muscle}_r_tendon` and `{muscle}-Pn_r` site names, scope materials as `myohead_matskin`, and clarify that 24 hinges are the coupled realization of 6 independent cervical DoFs.

## 2026-10-03 — Remove neck-preview scene backgrounds
Changed: scripts/render_composed_models.py, docs/images/models/myohead.png, docs/images/models/myofullbody_neck.png
Why: Render the two neck previews with transparent backgrounds and hide scene props/reflections without changing model assets.

## 2026-10-03 — Show the muscular neck in the model gallery
Changed: README.md, myo_sim/models/head/README.md, myo_sim/models/head/assets/myohead_assets.xml, docs/images/models/myohead.png, docs/images/models/myofullbody_neck.png
Why: Match the torso's tendon rendering width, document both full-body build variants and display the optional neck muscles without changing default full-body behavior.

## 2026-10-03 — Keep validation outside the installable package
Changed: docs/validation/neck/, myo_sim/models/head/, tests/test_head_build.py, scripts/, docs/wiki/testing-guide.md, docs/wiki/repository-map.md, docs/wiki/build-and-composition.md
Why: Retain a concise literature report and comparison figure in documentation, remove reproducibility archives/tooling, and keep only model assets, required licensing and runtime composition tests in the package changes.

## 2026-10-03 — Correct the prone neck-release benchmark
Changed: scripts/import_neck_source.py, scripts/validate_neck_dynamics.py, tests/test_neck_dynamics.py, myo_sim/models/head/assets/myohead_chain.xml, myo_sim/models/head/validation/, myo_sim/models/head/README.md, docs/images/validation/, docs/wiki/testing-guide.md
Why: Restore source passive mechanics, use anatomical fiber estimates and the published neck muscle correspondence, then calibrate graded feedback on nine participants and evaluate eight separately; keep measurements and acceptance bounds fixed.

## 2026-10-03 — Compare neck release dynamics with experimental trajectories
Changed: scripts/validate_neck_dynamics.py, tests/test_neck_dynamics.py, myo_sim/models/head/validation/, myo_sim/models/head/README.md, docs/images/validation/neck_dynamics.png, docs/wiki/testing-guide.md
Why: Add a checksum-pinned literature benchmark and unfit forward simulations; distinguish numerical correctness from failed prone biological agreement without altering production model parameters.

## 2026-10-03 — Validate the opt-in muscular neck in MyoFullBody
Changed: myo_sim/models/head/, myo_sim/build/head.py, myo_sim/build/compose.py, scripts/import_neck_source.py, scripts/validate_neck.py, tests/test_neck_validation.py, tests/test_build_registry.py, docs/wiki/build-and-composition.md
Why: Preserve the default full-body model while reproducing source kinematics and published neutral isometric neck strength in an opt-in assembly.
