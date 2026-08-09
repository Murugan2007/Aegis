# Contributing to Project FINDER-Lite

Thanks for considering a contribution — this project started as a hackathon
proof-of-concept and is meant to grow as an open teaching/prototyping
platform for SAR sensor-fusion. Contributions of all sizes are welcome.

## Ways to Contribute

-  **Bug reports** — especially firmware/hardware issues (radar parsing
  edge cases, servo jitter, ESP32 core version incompatibilities). Real
  hardware testing on different module batches is genuinely valuable since
  Hi-Link has shipped minor protocol variations across production runs.
-  **Hardware validation reports** — if you build this with different
  sensors, MCUs, or a different ESP32 core version, please share what
  worked/didn't. This helps everyone else building from this repo.
- **Feature contributions** — UWB radar integration, STM32 firmware port,
  MAVLink attitude fusion, acoustic localization, occlusion-aware
  simulation, etc. See the [Roadmap](README.md#-roadmap) for ideas.
- 📖 **Documentation** — clarifications, better wiring diagrams, translated
  docs, corrected protocol details.

## Before You Start

1. **Check existing issues** to avoid duplicate work.
2. **For larger changes** (new sensor support, architecture changes), open
   an issue first to discuss the approach before investing significant time.
3. **Keep the fusion logic honest** — if you add a capability claim, make
   sure it's backed by actual test data, not just "should work in theory."
   This project deliberately avoids overclaiming (see README limitations
   section) and PRs should hold to that standard.

## Submitting Changes

1. Fork the repo and create a branch: `git checkout -b feature/your-feature-name`
2. Make your changes. Keep commits focused and reasonably atomic.
3. If you touched firmware (`.ino`) — confirm it actually compiles in
   Arduino IDE before opening the PR, and note your ESP32 core version in
   the PR description (core version bumps have broken API compatibility
   before, e.g. `WebServer::send()` losing its 4-argument overload in core
   3.3.x).
4. If you touched the simulation (`sar_sim.py`) — run it headlessly to
   confirm no exceptions:
   ```bash
   python -c "import matplotlib; matplotlib.use('Agg'); exec(open('simulation/sar_sim.py').read().replace('plt.show()','pass'))"
   ```
5. Open a Pull Request with a clear description of what changed and why.

## Code Style

- **Firmware (C/C++):** match the existing style in `FinderLite_ESP32.ino`
  — descriptive names, comments explaining *why* for any protocol-specific
  magic numbers (byte offsets, sign encoding, etc.), not just *what*.
- **Python:** standard PEP 8-ish formatting is fine; prioritize readability
  over cleverness, especially in the fusion-logic functions since those are
  meant to be read and ported by people unfamiliar with the codebase.

## Reporting Issues

Please include:
- Hardware used (exact module part numbers/revisions if known)
- ESP32 board package / Arduino core version, or Python/matplotlib version
- Full error message or unexpected behavior description
- Steps to reproduce

## Code of Conduct

Be respectful, be constructive, assume good faith. This project exists to
help people learn and build — keep discussions focused on that.
