"""
animated_eyes.py — EVE-style glowing pill eyes (Wall-E aesthetic)

Controls:
  E        — excited
  D        — disappointed
  B        — blink
  ESC / Q  — quit

Platform:
  Windows  → 480×320 windowed (dev preview)
  Linux    → fullscreen (Raspberry Pi deployment)
"""

import sys
import math
import random
import pygame
import pygame.gfxdraw

# ── Display ───────────────────────────────────────────────────────────────────
WIDTH, HEIGHT = 480, 320
FPS           = 40
FULLSCREEN    = sys.platform == "linux"

# ── Palette ───────────────────────────────────────────────────────────────────
BG_COLOR      = (10,  10,  20)       # deep navy-black
EYE_COLOR     = (0,  212, 255)       # #00D4FF cyan core

# ── Eye geometry ──────────────────────────────────────────────────────────────
EYE_RX       = 72    # base half-width  (pixels)
EYE_RY       = 30    # base half-height (pixels)
EYE_LEFT_CX  = 136   # left  eye centre x
EYE_RIGHT_CX = 344   # right eye centre x  (208px separation)
EYE_CY       = 155   # both eyes centre y
EYE_ANGLE    = 12    # tilt in degrees — inner corners up, outer corners down (V shape)

# ── Glow ──────────────────────────────────────────────────────────────────────
GLOW_LAYERS = 14
GLOW_STEP   = 7      # extra px per glow layer  (14×7 = 98px max halo)
# Per-ring SRCALPHA blits accumulate toward the core.  These values are tuned so
# the cumulative zone brightness ≈ 70 % at the core edge in normal mode (clearly
# visible soft glow), falling to ~12 % at 49 px out and near-zero at 98 px.
GLOW_ALPHAS = (70, 54, 41, 31, 23, 17, 13, 9, 7, 5, 4, 3, 2, 1)  # innermost → outermost


# ── Drawing helpers ───────────────────────────────────────────────────────────

def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _crescent_pts(ecx: int, ecy: int, rx: int, ry: int,
                   crescent: float, hood: float = 0.0, n: int = 48) -> list:
    """Polygon points for an eye shape with optional crescent morphs.

    crescent: bottom edge rises toward centre (happy squint, used for excited).
    hood:     top edge drops toward centre (heavy-lidded stern look, used for disappointed).
    Both can be combined; values 0=ellipse, 1=half-dome.
    """
    pts = []
    for i in range(n):
        a  = 2 * math.pi * i / n
        px = ecx + rx * math.cos(a)
        s  = math.sin(a)          # >0 = bottom half in screen coords (y down)
        if s > 0:
            py = ecy + ry * s * (1.0 - crescent)
        else:
            py = ecy + ry * s * (1.0 - hood)
        pts.append((int(px), int(py)))
    return pts


def draw_eye(dest: pygame.Surface, cx: int, cy: int,
             rx: int = EYE_RX, ry: int = EYE_RY,
             angle: float = 0.0, glow_mult: float = 1.0,
             crescent: float = 0.0, hood: float = 0.0) -> None:
    """Draw a single glowing EVE pill eye centred at (cx, cy)."""
    r, g, b = EYE_COLOR

    max_rx = rx + GLOW_STEP * GLOW_LAYERS
    max_ry = ry + GLOW_STEP * GLOW_LAYERS
    diag   = int(math.hypot(max_rx, max_ry)) + 4
    size   = diag * 2
    ecx, ecy = size // 2, size // 2

    # ── Glow rings: one SRCALPHA surface per ring, each rotated and blitted ───
    # SRCALPHA blitted to an opaque dest blends (not overwrites), so the second
    # eye's glow never cuts into the first eye. Separate surfaces also avoid the
    # alpha-stacking band artifact that occurs when multiple shapes share one surface.
    use_poly = crescent > 0.01 or hood > 0.01
    for i in range(GLOW_LAYERS - 1, -1, -1):
        grx   = rx + GLOW_STEP * (i + 1)
        gry   = ry + GLOW_STEP * (i + 1)
        alpha = min(255, int(GLOW_ALPHAS[i] * glow_mult))
        ring  = pygame.Surface((size, size), pygame.SRCALPHA)
        if use_poly:
            pts = _crescent_pts(ecx, ecy, grx, gry, crescent, hood)
            pygame.gfxdraw.filled_polygon(ring, pts, (r, g, b, alpha))
        else:
            pygame.gfxdraw.filled_ellipse(ring, ecx, ecy, grx, gry, (r, g, b, alpha))
        rot_ring = pygame.transform.rotate(ring, angle)
        rw, rh   = rot_ring.get_size()
        dest.blit(rot_ring, (cx - rw // 2, cy - rh // 2))

    # ── Core ──────────────────────────────────────────────────────────────────
    core = pygame.Surface((size, size), pygame.SRCALPHA)
    if use_poly:
        pts = _crescent_pts(ecx, ecy, rx, ry, crescent, hood)
        pygame.gfxdraw.filled_polygon(core, pts, (r, g, b, 255))
    else:
        pygame.gfxdraw.filled_ellipse(core, ecx, ecy, rx, ry, (r, g, b, 255))
    rotated = pygame.transform.rotate(core, angle)
    rw, rh  = rotated.get_size()
    dest.blit(rotated, (cx - rw // 2, cy - rh // 2))


# ── Blink constants ──────────────────────────────────────────────────────────
BLINK_CLOSE_SPEED  = 16.0   # lerp speed while closing (fast)
BLINK_OPEN_SPEED   = 8.5    # lerp speed while opening (slower)
BLINK_CLOSED_FRAC  = 0.04   # eye height fraction at peak of blink
BLINK_CLOSE_THRESH = 0.10   # switch to opening once below this
BLINK_OPEN_THRESH  = 0.97   # return to idle once above this

# ── Excited constants ─────────────────────────────────────────────────────────
EXCITED_SCALE_X          = 1.04    # barely-there horizontal stretch
EXCITED_SCALE_Y          = 1.06    # barely-there vertical stretch
EXCITED_GLOW             = 1.75    # bright glow bloom
EXCITED_CRESCENT         = 0.72    # bottom-rises amount (0=ellipse, 1=half-dome)
EXCITED_DURATION         = 3.5     # seconds before returning to normal
EXCITED_INTRO_BLINKS     = 3       # rapid flutter blinks at start
EXCITED_INTRO_CLOSE_TIME = 0.030   # seconds to snap closed per intro blink
EXCITED_INTRO_OPEN_TIME  = 0.050   # seconds to snap open per intro blink
PARAM_LERP_SPEED         = 9.0     # speed for scale/glow/crescent transitions

# ── Disappointed constants ────────────────────────────────────────────────────
DISAPP_SCALE_X    = 1.0     # no horizontal change
DISAPP_SCALE_Y    = 1.0     # full height — hood does the narrowing visually
DISAPP_GLOW       = 1.0     # dimmer than normal (1.5)
DISAPP_HOOD       = 0.68    # top edge drops toward centre (0=ellipse, 1=half-dome from top)
DISAPP_DURATION   = 3.5     # seconds before returning to normal


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    pygame.init()

    flags  = (pygame.FULLSCREEN | pygame.NOFRAME) if FULLSCREEN else 0
    screen = pygame.display.set_mode((WIDTH, HEIGHT), flags | pygame.DOUBLEBUF)
    pygame.display.set_caption("EVE Eyes")
    pygame.mouse.set_visible(False)

    clock = pygame.time.Clock()

    # ── Blink state ───────────────────────────────────────────────────────────
    open_frac   = 1.0
    blink_phase = 'idle'       # 'idle' | 'closing' | 'opening'
    blink_timer = random.uniform(2.5, 5.0)

    # ── Expression animated params (all lerp toward their targets) ────────────
    scale_x    = 1.0;  scale_x_tgt    = 1.0
    scale_y    = 1.0;  scale_y_tgt    = 1.0
    glow_mult  = 1.0;  glow_mult_tgt  = 1.0
    crescent   = 0.0;  crescent_tgt   = 0.0
    hood       = 0.0;  hood_tgt       = 0.0
    angle_mult = 1.0;  angle_mult_tgt = 1.0

    # ── Expression state ──────────────────────────────────────────────────────
    expr_state       = 'normal'   # 'normal' | 'excited_intro' | 'excited' | 'disappointed'
    expr_timer       = 0.0
    intro_blinks     = 0          # flutter blinks completed so far
    intro_blink_dir  = 'closing'  # 'closing' | 'opening'
    intro_blink_time = 0.0        # countdown for current intro blink phase

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        # ── Events ────────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_b and blink_phase == 'idle':
                    blink_phase = 'closing'
                elif event.key == pygame.K_e:
                    expr_state       = 'excited_intro'
                    expr_timer       = EXCITED_DURATION
                    intro_blinks     = 0
                    intro_blink_dir  = 'closing'
                    intro_blink_time = EXCITED_INTRO_CLOSE_TIME
                    open_frac        = 1.0
                elif event.key == pygame.K_d:
                    expr_state = 'disappointed'
                    expr_timer = DISAPP_DURATION

        # ── Expression state machine ──────────────────────────────────────────
        if expr_state == 'excited_intro':
            # Timer-based rapid flutter: each phase has a fixed duration
            scale_x_tgt    = EXCITED_SCALE_X
            scale_y_tgt    = EXCITED_SCALE_Y
            glow_mult_tgt  = EXCITED_GLOW
            hood_tgt       = 0.0
            angle_mult_tgt = 1.0
            intro_blink_time -= dt
            if intro_blink_dir == 'closing':
                # Linearly drive to closed over CLOSE_TIME
                open_frac = max(0.04, open_frac - dt / EXCITED_INTRO_CLOSE_TIME)
                if intro_blink_time <= 0:
                    intro_blink_dir  = 'opening'
                    intro_blink_time = EXCITED_INTRO_OPEN_TIME
            else:
                open_frac = min(1.0, open_frac + dt / EXCITED_INTRO_OPEN_TIME)
                if intro_blink_time <= 0:
                    intro_blinks    += 1
                    open_frac        = 1.0
                    intro_blink_dir  = 'closing'
                    intro_blink_time = EXCITED_INTRO_CLOSE_TIME
            if intro_blinks >= EXCITED_INTRO_BLINKS:
                expr_state  = 'excited'
                blink_phase = 'idle'
                open_frac   = 1.0

        elif expr_state == 'excited':
            scale_x_tgt    = EXCITED_SCALE_X
            scale_y_tgt    = EXCITED_SCALE_Y
            glow_mult_tgt  = EXCITED_GLOW
            crescent_tgt   = EXCITED_CRESCENT
            hood_tgt       = 0.0
            angle_mult_tgt = 1.0
            expr_timer    -= dt
            if expr_timer <= 0:
                expr_state = 'normal'

        elif expr_state == 'disappointed':
            scale_x_tgt    = DISAPP_SCALE_X
            scale_y_tgt    = DISAPP_SCALE_Y
            glow_mult_tgt  = DISAPP_GLOW
            crescent_tgt   = 0.0
            hood_tgt       = DISAPP_HOOD
            angle_mult_tgt = 1.0
            expr_timer    -= dt
            if expr_timer <= 0:
                expr_state = 'normal'

        else:  # normal
            scale_x_tgt    = 1.0
            scale_y_tgt    = 1.0
            glow_mult_tgt  = 1.5
            crescent_tgt   = 0.0
            hood_tgt       = 0.0
            angle_mult_tgt = 1.0

        # ── Lerp animated params ──────────────────────────────────────────────
        t          = 1.0 - math.exp(-PARAM_LERP_SPEED * dt)
        scale_x    = lerp(scale_x,    scale_x_tgt,    t)
        scale_y    = lerp(scale_y,    scale_y_tgt,    t)
        glow_mult  = lerp(glow_mult,  glow_mult_tgt,  t)
        crescent   = lerp(crescent,   crescent_tgt,   t)
        hood       = lerp(hood,       hood_tgt,       t)
        angle_mult = lerp(angle_mult, angle_mult_tgt, t)

        # ── Blink state machine ───────────────────────────────────────────────
        if blink_phase == 'idle':
            open_frac    = lerp(open_frac, 1.0, 1.0 - math.exp(-BLINK_OPEN_SPEED * dt))
            blink_timer -= dt
            if blink_timer <= 0:
                blink_phase = 'closing'

        elif blink_phase == 'closing':
            open_frac = lerp(open_frac, BLINK_CLOSED_FRAC,
                             1.0 - math.exp(-BLINK_CLOSE_SPEED * dt))
            if open_frac < BLINK_CLOSE_THRESH:
                blink_phase = 'opening'

        elif blink_phase == 'opening':
            open_frac = lerp(open_frac, 1.0,
                             1.0 - math.exp(-BLINK_OPEN_SPEED * dt))
            if open_frac > BLINK_OPEN_THRESH:
                open_frac   = 1.0
                blink_phase = 'idle'
                blink_timer = random.uniform(2.5, 5.0)

        # ── Draw ──────────────────────────────────────────────────────────────
        rx_now = max(1, int(EYE_RX * scale_x))
        ry_now = max(1, int(EYE_RY * scale_y * open_frac))

        screen.fill(BG_COLOR)
        draw_eye(screen, EYE_LEFT_CX,  EYE_CY, rx=rx_now, ry=ry_now,
                 angle=-EYE_ANGLE * angle_mult, glow_mult=glow_mult, crescent=crescent, hood=hood)
        draw_eye(screen, EYE_RIGHT_CX, EYE_CY, rx=rx_now, ry=ry_now,
                 angle= EYE_ANGLE * angle_mult, glow_mult=glow_mult, crescent=crescent, hood=hood)
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
