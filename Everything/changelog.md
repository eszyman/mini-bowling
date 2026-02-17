# Everything Script - Changelog

## Version 1.1.0 - 2026-02-16

### Added
- `pin_config.h` — hardware pin assignments extracted from inline defines, matching the test script pattern
- `user_config.h` — all user-adjustable settings (servo angles, LED lengths, timing constants) extracted into a shared config file
- `DECK_LED_BRIGHTNESS` — separate brightness setting for deck LEDs, independent of lane LED brightness
- Optional `pin_config.user.h` and `user_config.user.h` overrides (git-ignored) for per-machine customization

### Changed
- Simplified raise servo config to match sweep servo pattern: define one angle, compute the mirror (R = 180 - L)

## Version 1.0 (Rev67) - 2026-02-14

Original files by Danny Lum (Rev67), moved to a new GitHub repository as version 1.0.
