---
name: social-testing-research
description: Read before adding reactive people to the sim, new social scores to course.py, or designing a pilot with real people: tools, simulators, datasets and metrics
metadata:
  type: reference
  updated: 2026-11-15T22:27:14Z
  covers: []
---

Web research 2026-11-15, not tried by us; overview in [[social-behavior-research]]. Our scoring: [[course-runs]], [[behavior-loop]]. Licences unconfirmed unless stated; the borrowing rule: [[external-dependencies]].

Findings
- ROS4HRI (REP-155, https://github.com/ros4hri): hri_msgs, hri_person_manager and others, Apache-2.0, Humble branches, Foxy unconfirmed. Useful for its per-person topic layout (`/humans/persons/<id>/...`), not to install.
- HuNavSim (https://github.com/robotics-upo/hunav_sim, README says MIT, check the LICENSE file; tested on Humble): simulated people move by the Social Force Model with behaviour trees and react to the robot as regular, impassive, surprised, curious, scared or threatening. Simulator-agnostic through ROS 2 services; wrappers for Gazebo, Isaac Sim, Webots, none for MuJoCo. Its hunav_evaluator computes distances to people, intimate, personal and social space intrusions, collisions, time not moving, jerk and social work. lightsfm (BSD-3) is its small social-force library.
- Others: SEAN 2.0 (Unity, Yale); SocNavBench (replays recorded people, so they don't react, like ours); pedsim_ros (libpedsim LGPL, old ROS 1); Arena 4.0/5.0 (heavy, navigation benchmarks).
- Datasets for human motion: JRDB, JRDB-Social, THÖR, SCAND, MuSoHu (licences not found); ATC approach set ([[engagement-research]]).
- Evaluation: Francis et al. 2025, eight principles (safety, comfort, legibility, politeness, social competence, understanding other agents, proactivity, context) and a metric taxonomy (https://arxiv.org/abs/2306.16740). Questionnaires: RoSAS (18 items: warmth, competence, discomfort), stronger than Godspeed; NARS for attitudes beforehand. Revealing a Wizard-of-Oz operator lowered perceived intelligence, so debrief after the ratings.
- G1 projects with conversation, examples only, no licences checked: SaxionMechatronics/unitree_converse (on-robot wake word, speech-to-text, Ollama, Piper), RafiMAA/Unitree_G1_EDU_Robot (Nav2 and a voice agent), JeanMayoko18/g1_social_navigation (Foxy and Humble, no licence: read only).

Biggest gap: our sim people can't react, so a score can't tell good behaviour from luck. Add optional reactions to `harp/movers.py`'s people, written ourselves after HuNavSim's types: ignore, stop and look, come closer, back off, block, plus accept or decline a talk with some probability. Run each scenario over several seeds.

Scores to add to `course.py people`: closest and mean distance to each person, time in each Hall zone, approach angle to the target, speed when entering 3.6 m, time to first greeting, success, length and decline rate, whether it backs off after a decline, share of the talk spent facing the person, unintended approaches to bystanders. Report mean and spread over seeds, grouped by Francis's principles.

Pilot: 10 to 15 people, within-subject, order counterbalanced; our loop against a Wizard-of-Oz operator and against standing and waiting; RoSAS plus Godspeed safety and likeability and a free comment; the same logged metrics; treat it as descriptive.
