"""
2D Collision Visual Debugger
Controls:
  Left-drag        move shape
  Scroll           rotate selected shape
  SPACE            toggle physics simulation
  G                toggle gravity
  1                spawn circle
  2                spawn box
  DEL              delete selected shape
  R                reset to default scene
  [ / ]            decrease / increase restitution of selected
  Shift+[ / ]      decrease / increase friction of selected
"""
import math
import sys

import pygame

from box import Box
from circle import Circle
from collision import Collide, CollisionPair, CollisionResult
from resolution import Resolution
from vector2 import Vec2

# --- constants ----------------------------------------------------------------

SCALE = 60          # pixels per world unit
WIDTH, HEIGHT = 960, 720
FPS = 60
STATIC_MASS = 1e10  # objects with this mass are treated as immovable

BG           = (20,  20,  30)
GRID_DIM     = (32,  32,  45)
AXIS_DIM     = (55,  55,  75)
SHAPE_CLR    = (70,  160, 240)
COLLIDE_CLR  = (220, 60,  60)
SELECT_CLR   = (140, 240, 100)
STATIC_CLR   = (110, 110, 130)
NORMAL_CLR   = (255, 210, 50)
CONTACT_CLR  = (255, 100, 100)
VEL_CLR      = (100, 180, 255)
TEXT_CLR     = (190, 190, 200)

# --- coordinate helpers -------------------------------------------------------

def to_screen(pos: Vec2) -> tuple[int, int]:
    return (int(pos.x * SCALE + WIDTH / 2), int(-pos.y * SCALE + HEIGHT / 2))


def to_world(sx: int, sy: int) -> Vec2:
    return Vec2((sx - WIDTH / 2) / SCALE, -(sy - HEIGHT / 2) / SCALE)


def is_static(obj) -> bool:
    return obj.mass >= STATIC_MASS * 0.9

# --- drawing helpers ----------------------------------------------------------

def draw_arrow(surface, color, start: tuple, end: tuple, width=2, tip=10):
    if start == end:
        return
    pygame.draw.line(surface, color, start, end, width)
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1:
        return
    dx, dy = dx / length, dy / length
    left  = (end[0] - tip * dx + tip * 0.5 * dy, end[1] - tip * dy - tip * 0.5 * dx)
    right = (end[0] - tip * dx - tip * 0.5 * dy, end[1] - tip * dy + tip * 0.5 * dx)
    pygame.draw.polygon(surface, color, [end, left, right])


def draw_circle_shape(surface, obj: Circle, color):
    cx, cy = to_screen(obj.position)
    r = max(2, int(obj.radius * SCALE))
    pygame.draw.circle(surface, color, (cx, cy), r, 2)
    ex = cx + int(math.cos(obj.rotation) * r)
    ey = cy - int(math.sin(obj.rotation) * r)
    pygame.draw.line(surface, color, (cx, cy), (ex, ey), 1)


def draw_box_shape(surface, obj: Box, color):
    hw, hh = obj.halfWidth, obj.halfHeight
    corners = [Vec2(-hw, -hh), Vec2(hw, -hh), Vec2(hw, hh), Vec2(-hw, hh)]
    pts = [to_screen(obj.position + c.rotate(obj.rotation)) for c in corners]
    pygame.draw.polygon(surface, color, pts, 2)
    # Draw center dot
    cx, cy = to_screen(obj.position)
    pygame.draw.circle(surface, color, (cx, cy), 3)

# --- main viewer class --------------------------------------------------------

class DebugViewer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("2D Collision Debugger")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 14)

        self.objects: list = []
        self.selected = None    # highlighted for info panel
        self.dragging = None    # currently being dragged
        self.drag_offset = Vec2(0, 0)
        self.simulating = False
        self.gravity_on = False

        self._reset()

    # --- scene management -----------------------------------------------------

    def _reset(self):
        self.objects = [
            Circle(position=Vec2(-3.0, 0.0), velocity=Vec2(0, 0),
                   mass=1.0, radius=1.0),
            Circle(position=Vec2(3.0, 0.5), velocity=Vec2(0, 0),
                   mass=1.5, radius=1.3),
            Box(position=Vec2(0.0, -3.8), velocity=Vec2(0, 0),
                mass=STATIC_MASS, halfWidth=7.0, halfHeight=0.4),
        ]
        self.selected = None
        self.dragging = None

    def _spawn_circle(self):
        self.objects.append(
            Circle(position=Vec2(0, 3), mass=1.0, radius=0.8)
        )

    def _spawn_box(self):
        self.objects.append(
            Box(position=Vec2(0, 3), mass=1.0, halfWidth=1.0, halfHeight=0.6)
        )

    # --- hit testing ----------------------------------------------------------

    def _hit_test(self, wp: Vec2):
        for obj in reversed(self.objects):
            if isinstance(obj, Circle):
                if (wp - obj.position).length() <= obj.radius:
                    return obj
            elif isinstance(obj, Box):
                local = (wp - obj.position).rotate(-obj.rotation)
                if abs(local.x) <= obj.halfWidth and abs(local.y) <= obj.halfHeight:
                    return obj
        return None

    # --- physics step ---------------------------------------------------------

    def _step(self, dt: float):
        gravity = Vec2(0, -9.8)
        for obj in self.objects:
            if obj is self.dragging or is_static(obj):
                continue
            if self.gravity_on:
                obj.velocity = obj.velocity + gravity * dt
            obj.position = obj.position + obj.velocity * dt
            obj.rotation += obj.angVelocity * dt

        for i in range(len(self.objects)):
            for j in range(i + 1, len(self.objects)):
                a, b = self.objects[i], self.objects[j]
                try:
                    result = Collide(a, b)
                except KeyError:
                    continue
                if not result.colliding:
                    continue
                Resolution(CollisionPair(a, b, result))
                # Baumgarte positional correction
                inv_a = 0.0 if is_static(a) else 1.0 / a.mass
                inv_b = 0.0 if is_static(b) else 1.0 / b.mass
                total = inv_a + inv_b
                if total > 0:
                    correction = result.normal * (result.overlap / total) * 0.6
                    a.position = a.position - correction * inv_a
                    b.position = b.position + correction * inv_b

    # --- collision query (for display) ----------------------------------------

    def _query_collisions(self) -> list[tuple[int, int, CollisionResult]]:
        out = []
        for i in range(len(self.objects)):
            for j in range(i + 1, len(self.objects)):
                try:
                    r = Collide(self.objects[i], self.objects[j])
                    out.append((i, j, r))
                except KeyError:
                    pass
        return out

    # --- drawing --------------------------------------------------------------

    def _draw_grid(self):
        for x in range(-20, 21):
            sx, _ = to_screen(Vec2(x, 0))
            pygame.draw.line(self.screen, AXIS_DIM if x == 0 else GRID_DIM,
                             (sx, 0), (sx, HEIGHT))
        for y in range(-15, 16):
            _, sy = to_screen(Vec2(0, y))
            pygame.draw.line(self.screen, AXIS_DIM if y == 0 else GRID_DIM,
                             (0, sy), (WIDTH, sy))

    def _draw_hud(self, collisions: list):
        y = 10

        def blit(text, color=TEXT_CLR):
            nonlocal y
            self.screen.blit(self.font.render(text, True, color), (10, y))
            y += 18

        blit(f"[SPACE] Sim: {'ON ' if self.simulating else 'OFF'}   "
             f"[G] Gravity: {'ON ' if self.gravity_on else 'OFF'}")
        blit("[Drag] move   [Scroll] rotate   [DEL] delete   [R] reset")
        blit("[1] add circle   [2] add box")
        blit("[[ / ]] restitution   [Shift+[ / ]] friction  (select first)")

        if collisions:
            blit("")
            for i, j, r in collisions:
                if r.colliding:
                    blit(f"obj{i} <-> obj{j}  "
                         f"overlap={r.overlap:.3f}  "
                         f"n=({r.normal.x:.2f}, {r.normal.y:.2f})",
                         COLLIDE_CLR)

        # Selected object info panel (bottom-left)
        if self.selected:
            obj = self.selected
            mass_str = "static" if is_static(obj) else f"{obj.mass:.2f}"
            lines = [
                "--- Selected ---",
                f"pos ({obj.position.x:.2f}, {obj.position.y:.2f})",
                f"vel ({obj.velocity.x:.2f}, {obj.velocity.y:.2f})",
                f"mass {mass_str}   restitution {obj.restitution:.2f}   friction {obj.frictionCoeff:.2f}",
                f"rotation {math.degrees(obj.rotation):.1f} deg   angVel {math.degrees(obj.angVelocity):.1f} deg/s",
            ]
            if isinstance(obj, Circle):
                lines.append(f"radius {obj.radius:.2f}")
            elif isinstance(obj, Box):
                lines.append(f"size {obj.halfWidth*2:.2f} x {obj.halfHeight*2:.2f}")
            base_y = HEIGHT - len(lines) * 18 - 10
            for k, line in enumerate(lines):
                self.screen.blit(
                    self.font.render(line, True, SELECT_CLR),
                    (10, base_y + k * 18)
                )

    # --- main loop ------------------------------------------------------------

    def run(self):
        while True:
            dt = min(self.clock.tick(FPS) / 1000.0, 0.05)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.simulating = not self.simulating
                    elif event.key == pygame.K_g:
                        self.gravity_on = not self.gravity_on
                    elif event.key == pygame.K_r:
                        self._reset()
                    elif event.key == pygame.K_1:
                        self._spawn_circle()
                    elif event.key == pygame.K_2:
                        self._spawn_box()
                    elif event.key == pygame.K_DELETE:
                        if self.selected and len(self.objects) > 1:
                            self.objects.remove(self.selected)
                            self.selected = None
                    elif event.key == pygame.K_LEFTBRACKET and self.selected:
                        if event.mod & pygame.KMOD_SHIFT:
                            self.selected.frictionCoeff = round(max(0.0, self.selected.frictionCoeff - 0.05), 2)
                        else:
                            self.selected.restitution = round(max(0.0, self.selected.restitution - 0.05), 2)
                    elif event.key == pygame.K_RIGHTBRACKET and self.selected:
                        if event.mod & pygame.KMOD_SHIFT:
                            self.selected.frictionCoeff = round(min(2.0, self.selected.frictionCoeff + 0.05), 2)
                        else:
                            self.selected.restitution = round(min(1.0, self.selected.restitution + 0.05), 2)

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        wp = to_world(*event.pos)
                        hit = self._hit_test(wp)
                        self.selected = hit
                        if hit:
                            self.dragging = hit
                            self.drag_offset = hit.position - wp

                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self.dragging = None

                elif event.type == pygame.MOUSEMOTION:
                    if self.dragging:
                        wp = to_world(*event.pos)
                        self.dragging.position = wp + self.drag_offset
                        self.dragging.velocity = Vec2(0, 0)
                        self.dragging.angVelocity = 0.0

                elif event.type == pygame.MOUSEWHEEL:
                    if self.selected:
                        self.selected.rotation += event.y * 0.1

            if self.simulating:
                self._step(dt)

            # ---- render ----
            self.screen.fill(BG)
            self._draw_grid()

            collisions = self._query_collisions()
            colliding_ids = set()
            for i, j, r in collisions:
                if r.colliding:
                    colliding_ids.add(id(self.objects[i]))
                    colliding_ids.add(id(self.objects[j]))
            selected_id = id(self.selected) if self.selected else None

            for obj in self.objects:
                if id(obj) == selected_id:
                    color = SELECT_CLR
                elif is_static(obj):
                    color = STATIC_CLR
                elif id(obj) in colliding_ids:
                    color = COLLIDE_CLR
                else:
                    color = SHAPE_CLR

                if isinstance(obj, Circle):
                    draw_circle_shape(self.screen, obj, color)
                elif isinstance(obj, Box):
                    draw_box_shape(self.screen, obj, color)

                # Velocity arrow
                if not is_static(obj) and obj.velocity.length() > 0.05:
                    s = to_screen(obj.position)
                    e = to_screen(obj.position + obj.velocity * 0.25)
                    draw_arrow(self.screen, VEL_CLR, s, e, 1, 6)

            # Collision overlays: normal arrow + contact points
            for i, j, r in collisions:
                if not r.colliding:
                    continue
                a, b = self.objects[i], self.objects[j]
                mid = Vec2((a.position.x + b.position.x) / 2,
                           (a.position.y + b.position.y) / 2)
                norm_end = mid + r.normal * max(r.overlap * 2.0, 0.6)
                draw_arrow(self.screen, NORMAL_CLR,
                           to_screen(mid), to_screen(norm_end), 2, 10)
                #cp0, cp1 = r.contactPoints
                for cp in r.contactPoints:
                    if cp.length() > 0.001: # or cp1.length() > 0.001:
                        pygame.draw.circle(self.screen, CONTACT_CLR, to_screen(cp), 5)
                    #pygame.draw.circle(self.screen, CONTACT_CLR, to_screen(cp1), 5)

            self._draw_hud(collisions)
            pygame.display.flip()


if __name__ == "__main__":
    viewer = DebugViewer()
    viewer.run()
