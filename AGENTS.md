# AGENTS.md — Development & Linting Guide

## Linting & Code Quality

Before committing, lint every file type you changed and fix what the linter reports. Commits should be made only after the relevant linters pass. Run `make lint` to check everything at once.

| File type | Linter | Run |
|---|---|---|
| Python (`.py`) | `ruff` / `flake8` | `ruff check ground_station/` & `flake8 ground_station/` |
| C++ (`.cpp`, `.h`) | `g++ -Wall -Wextra -Werror` | `make verify-firmware` |
| OpenSCAD (`.scad`) | `openscad` | `openscad -o /dev/null <model.scad>` |

### Quick Commands
```bash
# Check code style & linting
make lint

# Auto-format Python code
make format

# Run full test suite
make test

# Verify C++ firmware math & failsafe
make verify-firmware

# Run all checks at once
make check
```
