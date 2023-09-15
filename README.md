# CUGL Distributed Physics Simulations Library
by Barry Lyu, under the guidance of Professor Walker M. White. 

This repository contains the research codebase and preliminary API to be integrated into CUGL in the future. 

This demo is based on the Physics Demo for the Cornell University Game Library (CUGL)

The purpose of this library is to provide cross-platform, real-time, and interactive physics simulation that runs on different devices. Unlike traditional host-client models, this synchronization model does not need distinct servers, and has better simulation fidelity.

Features of the Distributed Physics Library include:
- Discrete physics simulations based on ticks
- Automatic object dynamics synchronization
- Physics body/Joint creation/deletion synchronization
- Fully customizable cross-device error correction scheme, with stock linear and PID interpolations.
- Dynamic transfer of authority.
- Built-in latency testing framework.
- Extensible event-based system for deterministic message communication.
- Network manager for game lobby creation and Complete API encapsulation for the networking layers.
