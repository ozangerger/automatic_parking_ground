#!/usr/bin/env python3
"""Builds docs/side_slip_kinematic_vs_dynamic.pptx (14 slides, 2 appendix slides, speaker notes).

    ~/.venvs/psim-slides/bin/python tools/slides/side_slip_figures.py   # figures, out/slides/side_slip/
    ~/.venvs/psim-slides/bin/python tools/slides/side_slip_deck.py

16:9, Calibri, a full-sentence title on every slide (like docs/ekf_states_and_odometry_scale.pptx).
The numbers come from side_slip_model.py: steady-state hand calculations with assumed parameters.
Nothing here is measured on the simulator, and the slides say so.
"""

import math
import re
import sys
from pathlib import Path

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Emu, Inches, Pt

sys.path.insert(0, str(Path(__file__).resolve().parent))
import side_slip_model as model  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
FIG = ROOT / "out" / "slides" / "side_slip"
OUT = ROOT / "docs" / "side_slip_kinematic_vs_dynamic.pptx"

INK, INK2, GRID, SURFACE = "0b0b0b", "52514e", "e4e3df", "f3f2ee"
BLUE, ORANGE, AQUA = "2a78d6", "eb6834", "1baf7a"
FONT = "Calibri"
SLIDE_W, SLIDE_H = 13.333, 7.5
MARGIN = 0.6
CONTENT_W = SLIDE_W - 2 * MARGIN
TITLE_H = 1.15
BODY_TOP = 1.45

P = model.P
warnings = []


def rgb(hex_color):
    return RGBColor.from_string(hex_color.upper())


# ---- text ---------------------------------------------------------------------------------------

TOKEN = re.compile(r"(_\{[^}]*\}|\^\{[^}]*\}|\*\*[^*]+\*\*)")


def add_runs(paragraph, text, size, color=INK, bold=False):
    """Text with _{sub}, ^{sup} and **bold** markup."""
    for part in TOKEN.split(text):
        if not part:
            continue
        run = paragraph.add_run()
        font = run.font
        font.name = FONT
        font.size = Pt(size)
        font.color.rgb = rgb(color)
        font.bold = bold
        if part.startswith("_{"):
            run.text = part[2:-1]
            run._r.get_or_add_rPr().set("baseline", "-25000")
        elif part.startswith("^{"):
            run.text = part[2:-1]
            run._r.get_or_add_rPr().set("baseline", "30000")
        elif part.startswith("**"):
            run.text = part[2:-2]
            font.bold = True
        else:
            run.text = part


def plain(text):
    return TOKEN.sub(lambda m: m.group(0)[2:-1] if not m.group(0).startswith("**") else m.group(0)[2:-2],
                     text)


def estimate_height(paragraphs, width_in, size, space_pt):
    """Rough height in inches of wrapped text (Calibri averages about 0.5 em per character)."""
    chars_per_line = max(1, int(width_in * 72 / (size * 0.5)))
    lines = 0
    for text in paragraphs:
        lines += max(1, math.ceil(len(plain(text)) / chars_per_line))
    return (lines * size * 1.2 + len(paragraphs) * space_pt) / 72 + 0.1


def set_bullet(paragraph, size, level=0):
    pPr = paragraph._p.get_or_add_pPr()
    indent = int(Pt(size) * 0.9)
    pPr.set("marL", str(indent * (level + 1)))
    pPr.set("indent", str(-indent))
    for child in list(pPr):
        if child.tag in (qn("a:buNone"), qn("a:buChar")):
            pPr.remove(child)
    bullet = pPr.makeelement(qn("a:buChar"), {"char": "•" if level == 0 else "–"})
    pPr.append(bullet)


def add_text(slide, x, y, w, h, paragraphs, size=18, color=INK, bullets=False, bold=False,
             space_pt=8, align=None, anchor=MSO_ANCHOR.TOP, name=""):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    frame = box.text_frame
    frame.word_wrap = True
    frame.vertical_anchor = anchor
    frame.margin_left = frame.margin_right = Inches(0.05)
    frame.margin_top = frame.margin_bottom = Inches(0.03)
    first = True
    texts = []
    for item in paragraphs:
        opts = {}
        if isinstance(item, tuple):
            item, opts = item
        paragraph = frame.paragraphs[0] if first else frame.add_paragraph()
        first = False
        psize = opts.get("size", size)
        add_runs(paragraph, item, psize, opts.get("color", color), opts.get("bold", bold))
        paragraph.space_after = Pt(opts.get("space", space_pt))
        if align is not None:
            paragraph.alignment = align
        if bullets and not opts.get("plain"):
            set_bullet(paragraph, psize, opts.get("level", 0))
        texts.append(item)
    needed = estimate_height(texts, w - 0.1 - (0.3 if bullets else 0), size, space_pt)
    if needed > h + 0.05:
        warnings.append(f"text may overflow ({needed:.2f} in needed, {h:.2f} in available): "
                        f"{plain(texts[0])[:50]!r} {name}")
    return box


def add_title(slide, text):
    box = add_text(slide, MARGIN, 0.35, CONTENT_W, TITLE_H, [text], size=28, bold=True,
                   anchor=MSO_ANCHOR.TOP, space_pt=0, name="(title)")
    return box


def add_footer(slide, number):
    add_text(slide, MARGIN, SLIDE_H - 0.45, 8, 0.3, ["parking_sim · side slip: kinematic and dynamic bicycle model"],
             size=10, color=INK2, space_pt=0)
    add_text(slide, SLIDE_W - MARGIN - 1.0, SLIDE_H - 0.45, 1.0, 0.3, [str(number)], size=10,
             color=INK2, align=PP_ALIGN.RIGHT, space_pt=0)


def add_picture(slide, name, x, y, max_w, max_h):
    path = FIG / name
    with Image.open(path) as image:
        px_w, px_h = image.size
    scale = min(max_w / px_w, max_h / px_h)
    w, h = px_w * scale, px_h * scale
    slide.shapes.add_picture(str(path), Inches(x + (max_w - w) / 2), Inches(y), Inches(w), Inches(h))
    return h


def add_card(slide, x, y, w, h, heading, lines, accent):
    shape = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    shape.fill.solid()
    shape.fill.fore_color.rgb = rgb(SURFACE)
    shape.line.fill.background()
    bar = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y), Inches(0.09), Inches(h))
    bar.fill.solid()
    bar.fill.fore_color.rgb = rgb(accent)
    bar.line.fill.background()
    add_text(slide, x + 0.25, y + 0.12, w - 0.4, 0.6, [heading], size=18, bold=True, color=accent,
             space_pt=0, name="(card heading)")
    add_text(slide, x + 0.25, y + 0.75, w - 0.4, h - 0.85, lines, size=16, bullets=False, space_pt=6,
             name="(card)")


def cell_border(cell, color=GRID, width=9525):
    tcPr = cell._tc.get_or_add_tcPr()
    for tag in ("a:lnL", "a:lnR", "a:lnT", "a:lnB"):
        for old in tcPr.findall(qn(tag)):
            tcPr.remove(old)
    for tag in ("a:lnL", "a:lnR", "a:lnT"):
        ln = tcPr.makeelement(qn(tag), {"w": "0"})
        ln.append(ln.makeelement(qn("a:noFill"), {}))
        tcPr.append(ln)
    ln = tcPr.makeelement(qn("a:lnB"), {"w": str(width)})
    fill = ln.makeelement(qn("a:solidFill"), {})
    fill.append(fill.makeelement(qn("a:srgbClr"), {"val": color.upper()}))
    ln.append(fill)
    tcPr.append(ln)


def add_table(slide, x, y, widths, rows, size=14, header=True, row_h=None, name=""):
    n_rows, n_cols = len(rows), len(widths)
    heights = []
    for r, row in enumerate(rows):
        lines = 1
        for c, text in enumerate(row):
            cpl = max(1, int((widths[c] - 0.2) * 72 / (size * 0.5)))
            lines = max(lines, sum(max(1, math.ceil(len(plain(t)) / cpl)) for t in text.split("\n")))
        heights.append(max(row_h or 0.0, lines * size * 1.2 / 72 + 0.16))
    total_h = sum(heights)
    frame = slide.shapes.add_table(n_rows, n_cols, Inches(x), Inches(y), Inches(sum(widths)),
                                   Inches(total_h))
    table = frame.table
    table.first_row = header
    table.horz_banding = False
    for c, width in enumerate(widths):
        table.columns[c].width = Inches(width)
    for r, row in enumerate(rows):
        table.rows[r].height = Inches(heights[r])
        for c, text in enumerate(row):
            cell = table.cell(r, c)
            cell.margin_left = cell.margin_right = Inches(0.08)
            cell.margin_top = cell.margin_bottom = Inches(0.04)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            frame_text = cell.text_frame
            frame_text.word_wrap = True
            is_header = header and r == 0
            cell.fill.solid()
            cell.fill.fore_color.rgb = rgb("2b2a28" if is_header else "ffffff")
            for i, line in enumerate(text.split("\n")):
                paragraph = frame_text.paragraphs[0] if i == 0 else frame_text.add_paragraph()
                add_runs(paragraph, line, size, "ffffff" if is_header else INK,
                         bold=is_header or c == 0)
            cell_border(cell)
    return total_h


def add_notes(slide, text):
    slide.notes_slide.notes_text_frame.text = text


# ---- the deck -----------------------------------------------------------------------------------

prs = Presentation()
prs.slide_width = Emu(12191695)
prs.slide_height = Emu(6858000)
BLANK = prs.slide_layouts[6]
counter = [0]


def new_slide(title=None):
    slide = prs.slides.add_slide(BLANK)
    counter[0] += 1
    if title:
        add_title(slide, title)
        add_footer(slide, counter[0])
    return slide


K = model.understeer_gradient()
PARK, DRIVE = model.PARKING, model.DRIVE_BY
deg = math.degrees


def fmt_case(case):
    ay = case.ay
    return dict(ay=ay, ar=deg(model.slip_rear(ay)), af=deg(model.slip_front(ay)),
                ks=deg(K * ay), crab10=100 * 10 * model.slip_rear(ay), crab20=20 * model.slip_rear(ay),
                factor=model.yaw_rate_factor(case.v), rerr=model.yaw_rate_error(case.v, case.radius))


park, drive = fmt_case(PARK), fmt_case(DRIVE)

# 1 ----------------------------------------------------------------------------------------------
slide = new_slide()
add_text(slide, MARGIN, 2.2, CONTENT_W, 1.2, ["Side slip and tire slip"], size=48, bold=True,
         space_pt=0, name="(deck title)")
add_text(slide, MARGIN, 3.5, CONTENT_W, 1.2,
         ["What the kinematic and the dynamic bicycle model say about them, and what adding the "
          "dynamic one to parking_sim would involve"], size=24, color=INK2, space_pt=0)
add_text(slide, MARGIN, 5.6, CONTENT_W, 0.8,
         ["Design discussion · numbers are steady-state hand calculations with assumed sedan "
          "parameters, not simulator output"], size=14, color=INK2, space_pt=0)
add_notes(slide,
          "About 15 minutes for the 13 content slides, one minute each; the two appendix slides are "
          "for questions. The deck answers one question: what changes when the plant gets a dynamic "
          "bicycle model instead of the kinematic one, and is it worth it. Everything numerical here "
          "comes from closed-form steady-state formulas with typical sedan parameters, because the "
          "dynamic model is not built yet. I will say so again where it matters.")

# 2 ----------------------------------------------------------------------------------------------
slide = new_slide("Bottom line: the kinematic model has no tire slip, the dynamic one computes it, "
                  "and at parking speed it is tiny")
card_w, gap = (CONTENT_W - 2 * 0.3) / 3, 0.3
add_card(slide, MARGIN, BODY_TOP, card_w, 3.35, "Kinematic (today)",
         ["α_{f} = α_{r} = 0 by construction.", "Wheels roll without slip; the body side slip β at the "
          "CG is purely geometric.", "No forces, no grip limit."], BLUE)
add_card(slide, MARGIN + card_w + gap, BODY_TOP, card_w, 3.35, "Parking: 1 m/s, R = 5.3 m",
         [f"α_{{r}} = {park['ar']:.2f}°, α_{{f}} = {park['af']:.2f}°",
          f"Rear-axle crab ≈ {park['crab10']:.1f} cm per 10 m.",
          f"Extra steering {park['ks']:.2f}°; yaw rate off by {100 * (1 - park['factor']):.1f} %."], ORANGE)
add_card(slide, MARGIN + 2 * (card_w + gap), BODY_TOP, card_w, 3.35, "Drive-by: 5 m/s, R = 8 m",
         [f"α_{{r}} = {drive['ar']:.1f}°, α_{{f}} = {drive['af']:.1f}°",
          f"Rear-axle crab ≈ {drive['crab20']:.2f} m per 20 m.",
          f"Extra steering {drive['ks']:.1f}°; yaw rate off by {100 * (1 - drive['factor']):.1f} % "
          f"({drive['rerr']:.3f} rad/s)."], AQUA)
add_text(slide, MARGIN, 5.05, CONTENT_W, 1.7,
         ["S2–S4 should barely change on a dynamic plant (about 1 cm).",
          "The value is in higher-speed and low-friction stress scenarios, and in testing the ECU's "
          "kinematic assumptions (EKF steering update, controllers)."],
         size=20, bullets=True, space_pt=8)
add_notes(slide,
          "Three cards. Left: today's model. The tires never slip because the model forces the wheels "
          "to roll along their heading. Middle: our actual maneuvers. At 1 m/s on a 5.3 m turn the "
          "lateral acceleration is 0.19 m/s squared and the rear tire slips 0.08 degrees, which is "
          "about 1.4 cm of sideways drift over a 10 m path. Right: a drive-by at 5 m/s on an 8 m turn, "
          "where the slip is 1.3 degrees and the drift is almost half a meter over 20 m. So the "
          "honest expectation is that S2 to S4 will not change visibly; the reason to build it is the "
          "stress scenario and testing what the ECU assumes. Assumed parameters: 1500 kg, centre of "
          "gravity 1.2 m behind the front axle, axle stiffnesses 80 and 90 kN per rad.")

# 3 ----------------------------------------------------------------------------------------------
slide = new_slide("Body side slip and the tire slip angles are different angles")
height = add_picture(slide, "angles.png", MARGIN, BODY_TOP - 0.1, CONTENT_W, 3.1)
add_table(slide, MARGIN, BODY_TOP + height + 0.05, [2.6, 5.4, 4.13],
          [["Angle", "Between …", "Small-angle form (forward, left positive)"],
           ["β  body side slip", "the body x-axis and the velocity of a point (the CG here)",
            "β ≈ v_{y} / v_{x}"],
           ["α_{f}  front tire slip", "the front wheel heading and the velocity of its contact patch",
            "α_{f} ≈ δ − β − a·r / v_{x}"],
           ["α_{r}  rear tire slip", "the rear wheel heading (the body x-axis) and the velocity of its "
            "contact patch", "α_{r} ≈ −β + b·r / v_{x}"]], size=14)
add_notes(slide,
          "Three panels, one angle each, on the same car in a left turn; the angles are exaggerated as "
          "on a slippery surface. Rear tire slip: the rear wheel points along the body but the rear "
          "axle moves along a slightly different direction. Body side slip: the velocity of the centre "
          "of gravity against the body axis. Front tire slip: the wheel points along delta but moves "
          "along delta minus alpha f. The table gives the small-angle relations: a is the distance from "
          "the centre of gravity to the front axle, b to the rear axle, r the yaw rate. Left is positive.")

# 4 ----------------------------------------------------------------------------------------------
slide = new_slide("Tire slip is the cause of the lateral force; body side slip is the result")
add_text(slide, MARGIN, BODY_TOP, 6.6, 4.9,
         ["Tire slip α produces the lateral force: F_{y} ≈ C·α, saturating at μ·F_{z}.",
          "The forces change the lateral velocity v_{y} and the yaw rate r; β follows from the motion.",
          "Exact identity: α_{r} is minus the velocity angle of the rear axle. With the rear axle as "
          "the vehicle's reference point, **the rear tire slip is the crab of that point**.",
          "β at the CG mixes two things: the rear-axle crab and a geometric part from the yaw rate."],
         size=20, bullets=True, space_pt=12)
add_card(slide, 7.6, BODY_TOP + 0.1, 5.1, 3.4, "Relations",
         ["β_{CG} = β_{rear} + b·r / v_{x}", "β_{rear} = −α_{r}",
          "→ β_{CG} = −α_{r} + b·r / v_{x}",
          ("The b·r / v_{x} term is all that is left with no slip: the kinematic β.", {"size": 15,
                                                                                       "color": INK2})],
         BLUE)
add_notes(slide,
          "Cause and effect. Tires generate force from their slip angle; the force changes how the body "
          "moves; the body side slip is a consequence. For our vehicle model the reference point is "
          "the rear axle, so the rear tire slip is literally the direction error of that point: the "
          "vehicle crabs by minus alpha r. The side slip at the centre of gravity adds a geometric term "
          "b times yaw rate over speed. With no slip at all that term is the whole story, which is the "
          "kinematic model on the next slides.")

# 5 ----------------------------------------------------------------------------------------------
slide = new_slide("The kinematic bicycle forces the wheels to roll without slip")
add_picture(slide, "kinematic.png", MARGIN, BODY_TOP - 0.05, 6.5, 5.3)
add_text(slide, 7.4, BODY_TOP, 5.3, 4.9,
         ["Assumption: each axle's velocity is along its wheels, so the rear axle has v_{y} = 0 and "
          "α_{f} = α_{r} = 0.",
          "Motion: ẋ = v cos ψ, ẏ = v sin ψ, ψ̇ = v tan δ / L, v̇ = a.",
          "Yaw rate r = v tan δ / L; turn radius R = L / tan δ of the rear axle.",
          "No mass, no forces, no grip: any curvature is followed exactly.",
          "In parking_sim: KinematicBicycleModel, RK4 at 10 ms, rear-axle reference."],
         size=18, bullets=True, space_pt=9)
add_notes(slide,
          "This is what the plant does today. The two axle perpendiculars meet at the centre of "
          "rotation, which fixes yaw rate and radius from the steering angle alone. The rear axle "
          "radius is L over tan delta. There are no tire parameters at all, which is why the model is "
          "robust and cheap, and also why it can never slide.")

# 6 ----------------------------------------------------------------------------------------------
slide = new_slide("It still has a body side slip, but that one is geometric and says nothing about grip")
add_picture(slide, "beta_geometric.png", MARGIN, BODY_TOP, 6.7, 4.6)
add_text(slide, 7.6, BODY_TOP, 5.1, 4.9,
         ["β at the CG = atan(b·tan δ / L): "
          f"{deg(model.beta_cg_kinematic(0.3)):.1f}° at 0.3 rad of steering.",
          "The rear axle, our reference point, has β = 0 in this model.",
          "So \"side slip\" without a point and a model is ambiguous: a large kinematic β is not a "
          "sliding car.",
          "Only the dynamic model separates the geometric part from the slip part."],
         size=18, bullets=True, space_pt=10)
add_notes(slide,
          "A common confusion. If you plot the body side slip of the kinematic bicycle you get a "
          "non-zero angle, nearly 10 degrees at 0.3 rad of steering, while both tire slip angles are "
          "exactly zero. That angle is geometry: the centre of gravity sits ahead of the rear axle, "
          "which does not slip sideways. When we log side slip from the dynamic model we will log the "
          "tire angles next to it for exactly this reason.")

# 7 ----------------------------------------------------------------------------------------------
slide = new_slide("The dynamic bicycle computes the slip angles from its states")
height = add_picture(slide, "dynamic_loop.png", MARGIN, BODY_TOP - 0.05, CONTENT_W, 3.3)
add_text(slide, MARGIN, BODY_TOP + height + 0.1, CONTENT_W, 1.9,
         ["States: x, y, ψ, v_{x}, v_{y}, r (body frame at the CG); inputs: δ and the commanded "
          "acceleration (longitudinal channel unchanged).",
          "Parameters: m, I_{z}, a, b, C_{f}, C_{r}, μ. Axle loads F_{z} from the static weight split.",
          "A closed loop: slip changes force, force changes v_{y} and r, those change the slip."],
         size=17, bullets=True, space_pt=7)
add_notes(slide,
          "Three blocks in a loop. The slip angles come from the state: the velocity of each axle "
          "against its wheel direction. A linear tire turns slip into lateral force and clamps it at "
          "mu times the axle load. The body equations at the centre of gravity turn forces into "
          "lateral acceleration and yaw acceleration. The steering angle is the input. The longitudinal "
          "channel stays as it is: the actuator's acceleration command, no drivetrain.")

# 8 ----------------------------------------------------------------------------------------------
slide = new_slide("The kinematic model is the limit of infinite tire stiffness")
add_text(slide, MARGIN, BODY_TOP, 6.3, 4.9,
         ["As C → ∞ the slip must vanish, and the two no-slip constraints of slide 5 return.",
          "A finite C lets the constraints give: the slip angle is the amount of violation, α = F / C.",
          "Steady turn: α_{f} = m·a_{y}·b / (L·C_{f}), α_{r} = m·a_{y}·a / (L·C_{r}).",
          "The grip limit |F| ≤ μ·F_{z} adds what the kinematic model cannot: a car that no longer "
          "follows the commanded curvature."],
         size=18, bullets=True, space_pt=10)
add_card(slide, 7.2, BODY_TOP + 0.05, 5.5, 4.45, "Steady cornering",
         ["δ = L / R + α_{f} − α_{r} = L / R + K·a_{y}",
          f"K = (m / L)(b / C_{{f}} − a / C_{{r}}) = {K:.4f} rad per m/s² (understeer)",
          "r = (v·δ / L) / (1 + K·v² / L)",
          ("Yaw rate is slightly lower than the kinematic v·tan δ / L, and the rear axle crabs by "
           "−α_{r}.", {"size": 15, "color": INK2})], ORANGE)
add_notes(slide,
          "A useful way to see the relation: make the tires infinitely stiff and the slip disappears, "
          "leaving the kinematic constraints. With finite stiffness the slip angle is the force over "
          "the stiffness. In a steady turn, the lateral acceleration loads the axles in proportion to "
          f"the weight split, which gives the two closed-form slips. The steering needs K times a_y on "
          f"top of the geometric angle; with our assumed parameters K is {K:.4f} radians per m/s "
          "squared, a mild understeer. These formulas are also the analytic test for the model later.")

# 9 ----------------------------------------------------------------------------------------------
slide = new_slide("At parking speed the slip is about a tenth of a degree; it only matters at "
                  "several m/s")
add_picture(slide, "alpha_vs_ay.png", MARGIN, BODY_TOP - 0.05, 7.4, 4.9)
add_card(slide, 8.2, BODY_TOP, 4.5, 2.25, "Parking: 1 m/s, R = 5.3 m",
         [f"a_{{y}} = {park['ay']:.2f} m/s²; α_{{r}} = {park['ar']:.2f}°",
          f"crab ≈ {park['crab10']:.1f} cm per 10 m"], ORANGE)
add_card(slide, 8.2, BODY_TOP + 2.45, 4.5, 2.25, "Drive-by: 5 m/s, R = 8 m",
         [f"a_{{y}} = {drive['ay']:.1f} m/s²; α_{{r}} = {drive['ar']:.1f}°",
          f"crab ≈ {drive['crab20']:.2f} m per 20 m"], AQUA)
add_notes(slide,
          "In the linear range the slip depends only on the lateral acceleration, so one plot covers all "
          "speeds and radii. The grey band is our parking envelope: at most one meter per second on a "
          "radius of at least 4.4 meters. The dot inside it is the 0.08 degrees from slide 2. The "
          "linear tire model is only trustworthy to about 5 degrees; the friction limit for dry "
          "asphalt, mu g, is near 8.8 m/s squared. The two cases on the right are the same numbers as "
          "slide 2, so you can check them against the curve. This is the quantitative reason S2 to S4 "
          "will not change much.")

# 10 ---------------------------------------------------------------------------------------------
slide = new_slide("Side by side: what each model contains and what it costs")
add_table(slide, MARGIN, BODY_TOP,
          [2.1, 4.8, 5.23],
          [["", "Kinematic bicycle", "Dynamic bicycle"],
           ["States", "x, y, ψ, v", "x, y, ψ, v_{x}, v_{y}, r (+ 2 tire deflections in option B)"],
           ["Parameters", "L", "+ m, I_{z}, a, b, C_{f}, C_{r}, μ (+ relaxation length in B)"],
           ["Tire slip", "zero by construction", "computed from the states"],
           ["Lateral force", "none", "linear in the slip, clamped at μ·F_{z}"],
           ["Grip limit", "none: any curvature is followed", "understeer, then sliding beyond μ·g"],
           ["Reverse", "only the sign of v", "slip uses |v_{long}|; sign relations tested separately"],
           ["Standstill", "trivial", "needs deflection states or a blend to be well defined"],
           ["Integration", "RK4 at 10 ms, no issue", "stiff below about 0.4 m/s with slip-based forces (A)"],
           ["Right for", "low lateral acceleration: parking", "up to saturation; linear tire to about 5°"]],
          size=14)
add_notes(slide,
          "The comparison in one table. The point to take away is the integration row and the standstill "
          "row: the kinematic model has no trouble at zero speed, the dynamic one needs a design "
          "decision there. That is the next slide. The cost of the dynamic model is about a day of "
          "reasoning in parameters but only a few hours of code; see the roadmap at the end.")

# 11 ---------------------------------------------------------------------------------------------
slide = new_slide("Caveat 1: slip-based tire forces are singular and stiff at low speed")
add_picture(slide, "stiffness.png", MARGIN, BODY_TOP - 0.05, 7.4, 4.9)
add_text(slide, 8.2, BODY_TOP, 4.55, 5.0,
         [f"Fast mode ≈ (C_{{f}} + C_{{r}}) / (m·v): {model.lateral_mode_rate(1.0):.0f} 1/s at 1 m/s, "
          f"{model.lateral_mode_rate(0.1):.0f} 1/s at 0.1 m/s.",
          f"RK4 at 10 ms is unstable below {model.rk4_min_speed(P.dt):.2f} m/s. Our maneuvers end at "
          "0.1 m/s and change gear at rest.",
          "(A) blend with the kinematic model below a threshold: simple, but the threshold must sit "
          "under about 0.2 m/s to keep the model active.",
          "(B) tire deflection states with a relaxation length: a fixed spring rate, no switch. "
          "**Recommended.**"],
         size=16, bullets=True, space_pt=8)
add_notes(slide,
          "The first real caveat, and the main design decision. The slip angle divides by the forward "
          "speed, so at low speed the tire forces change very fast: the fastest mode scales as the "
          "axle stiffnesses over mass times speed. An explicit RK4 step of 10 ms cannot integrate that "
          f"below about {model.rk4_min_speed(P.dt):.1f} m/s, and our speed profile ends at 0.1 m/s with "
          "gear changes at standstill. Option A, blending with the kinematic model, works only if the "
          "blend speed is tiny. Option B gives each tire a lateral deflection state with a relaxation "
          "length; the force then comes from a spring of fixed stiffness, about 12 rad per second at any "
          "speed, including zero. It costs two states and one more parameter and behaves well at the "
          "gear changes, so I recommend it. A relaxation length of 0.4 m and a little damping are the "
          "assumed values.")

# 12 ---------------------------------------------------------------------------------------------
slide = new_slide("Caveat 2: the ECU's kinematic yaw-rate model stays good at parking speed and "
                  "breaks from about 3 m/s")
add_picture(slide, "ecu_error.png", MARGIN, BODY_TOP - 0.05, 7.0, 4.9)
add_text(slide, 7.9, BODY_TOP, 4.85, 5.0,
         ["The EKF's steering update predicts r = bias + v·tan δ / L. The dynamic yaw rate is "
          f"{100 * (1 - drive['factor']):.1f} % lower at 5 m/s ({drive['rerr']:.3f} rad/s) but only "
          f"{100 * (1 - park['factor']):.2f} % at 1 m/s.",
          "steer_model_sigma = 0.003 rad/s: exceeded from about 2.8 m/s. The gate then rejects "
          "updates or the filter blames the gyro bias.",
          "The IMU's a_{y} becomes the specific force v̇_{y} + v_{x}·r (the ECU does not use it yet). "
          "Wheel odometry is unchanged: the rear wheels roll at v_{x}.",
          "Dead reckoning drifts by the crab α_{r}."],
         size=16, bullets=True, space_pt=8)
add_notes(slide,
          "What changes on the ECU side without touching it. The EKF's steering update compares the "
          "gyro with v tan delta over L. With understeer the real yaw rate is a little lower, by the "
          "factor one over one plus K v squared over L. At parking speed that is 0.1 percent, far below "
          "the filter's assumed model sigma of 0.003 rad per second. At 5 m/s on an 8 m turn it is "
          "0.017 rad per second, so the filter would start rejecting steering updates or absorb the "
          "difference into the gyro bias. That is a good stress test of the fusion and exactly the "
          "kind of mismatch the controller comparison also lacks. Wheel odometry is unaffected because "
          "the rear wheels roll along the body axis.")

# 13 ---------------------------------------------------------------------------------------------
slide = new_slide("Caveat 3: the limits of the dynamic model and of these numbers")
col_w = (CONTENT_W - 0.4) / 2
add_text(slide, MARGIN, BODY_TOP, col_w, 0.45, ["Model"], size=20, bold=True, color=BLUE, space_pt=0)
add_text(slide, MARGIN, BODY_TOP + 0.5, col_w, 4.6,
         ["Linear tire valid only to about 5°; saturation is a clamp, not a Pacejka curve.",
          "No load transfer, roll or pitch: constant axle loads, mass and stiffness.",
          "Longitudinal channel stays \"commanded acceleration\": no drivetrain, no longitudinal or "
          "combined slip.",
          "Planar: no road grade or camber."],
         size=17, bullets=True, space_pt=9)
add_text(slide, MARGIN + col_w + 0.4, BODY_TOP, col_w, 0.45, ["Process"], size=20, bold=True,
         color=ORANGE, space_pt=0)
add_text(slide, MARGIN + col_w + 0.4, BODY_TOP + 0.5, col_w, 4.6,
         ["Parameters are typical sedan values, not identified from a vehicle.",
          "Every number in this deck is a steady-state hand calculation, not simulator output.",
          "Reverse driving flips the slip sign relations: tested against a mirrored forward case.",
          "Tire force at standstill exists only with deflection states.",
          "PLAN.md lists dynamic tire models as out of scope: amend it first."],
         size=17, bullets=True, space_pt=9)
add_notes(slide,
          "The limits. Left: the physics we leave out, which is standard for a single-track model. The "
          "linear tire is fine to about 5 degrees and then we clamp; that is enough to show the "
          "saturation behaviour but not to reproduce a real tire. Right: the process caveats. The "
          "parameters are assumed, the numbers here are hand calculations, and PLAN.md currently says "
          "dynamic tire models are out of scope, so we amend the plan before building.")

# 14 ---------------------------------------------------------------------------------------------
slide = new_slide("How we would validate it, and the steps to get there")
add_text(slide, MARGIN, BODY_TOP, 6.0, 0.45, ["Validation"], size=20, bold=True, color=BLUE, space_pt=0)
add_text(slide, MARGIN, BODY_TOP + 0.5, 6.0, 4.9,
         ["Kinematic limit: at low speed the circle radius equals L / tan δ within tolerance.",
          "Steady cornering matches the closed-form slips and the understeer gradient.",
          "Saturation: force never exceeds μ·F_{z}; standstill holds with steering applied.",
          "Reverse mirrors forward; α_{r} = −(rear-axle velocity angle) holds exactly.",
          "Regression: S2–S4 on the dynamic plant stay within the acceptance limits.",
          "Stress case: about 3–5 m/s or low μ, with a plant axis in sim_table."],
         size=16, bullets=True, space_pt=7)
add_text(slide, 7.0, BODY_TOP, 5.7, 0.45, ["Roadmap"], size=20, bold=True, color=ORANGE, space_pt=0)
add_table(slide, 7.0, BODY_TOP + 0.55, [0.5, 3.9, 1.3],
          [["#", "Work", "Time"],
           ["0", "Amend PLAN.md: scope line and a new optional step", "10 min"],
           ["1", "IVehicleModel, VehicleDynamicsParams, DynamicBicycleModel, unit tests", "1.5 h"],
           ["2", "Truth and IMU plumbing, truth_alpha_f / _r / _beta in the CSV", "0.5 h"],
           ["3", "Scenario field, runner wiring, --plant option", "0.5 h"],
           ["4", "Validation and stress scenarios, plant axis in sim_table", "1 h"]], size=14)
add_text(slide, 7.0, 5.9, 5.7, 0.9,
         ["Later, separately: a slip-aware EKF and controllers."], size=15, color=INK2, space_pt=0)
add_notes(slide,
          "The validation list doubles as the unit tests for the model. The two that matter most are "
          "the kinematic-limit test, which proves the dynamic model reduces to what we have, and the "
          "identity between the rear slip and the rear-axle velocity angle. The roadmap is four small "
          "steps of about three and a half hours in total, after a ten-minute plan amendment. Open "
          "decisions for the room: the purpose (parking realism or higher-speed stress), option A or B "
          "for low speed, whether the ECU stays kinematic for now, and whether typical sedan parameters "
          "are acceptable.")

# A1 ---------------------------------------------------------------------------------------------
slide = new_slide("Appendix A1: symbols and sign conventions")
add_table(slide, MARGIN, BODY_TOP, [1.7, 10.43],
          [["Symbol", "Meaning"],
           ["x, y, ψ", "rear-axle position and heading; x forward, y left, yaw counter-clockwise"],
           ["v_{x}, v_{y}, r", "body-frame velocity components at the CG and yaw rate (left and "
            "counter-clockwise positive)"],
           ["δ", "front wheel angle (left positive)"],
           ["a, b, L", "CG to front axle, CG to rear axle, wheelbase L = a + b"],
           ["α_{f}, α_{r}", "tire slip angles; positive when the lateral force points left"],
           ["β", "body side slip: angle of the velocity of a point against the body x-axis "
            "(β_{CG} at the CG, β_{rear} at the rear axle)"],
           ["C_{f}, C_{r}, μ", "axle cornering stiffnesses (N/rad), friction coefficient"],
           ["a_{y}, R, K", "lateral acceleration, radius of the rear axle's path, understeer gradient"],
           ["Reverse", "slip is defined with |v_{long}| of the wheel; relations flip sign (tested)"]],
          size=14)
add_notes(slide, "Reference slide for questions.")

# A2 ---------------------------------------------------------------------------------------------
slide = new_slide("Appendix A2: the assumed parameters behind every number")
add_table(slide, MARGIN, BODY_TOP, [3.6, 2.8, 5.73],
          [["Parameter", "Value", "Note"],
           ["Wheelbase L", f"{P.L} m", "the simulator's VehicleParams"],
           ["Mass m", f"{P.m:.0f} kg", "typical mid-size sedan, assumed"],
           ["CG to front / rear axle  a, b", f"{P.a} m, {P.b} m", "front-heavy, 55.6 % of the weight on the front"],
           ["Cornering stiffness C_{f}, C_{r}", f"{P.cf / 1e3:.0f}, {P.cr / 1e3:.0f} kN/rad", "per axle, assumed"],
           ["Friction coefficient μ", f"{P.mu}", f"dry asphalt; μ·g = {P.mu * model.G:.1f} m/s²"],
           ["Understeer gradient K", f"{K:.4f} rad per m/s²", "(m / L)(b / C_{f} − a / C_{r})"],
           ["EKF steer_model_sigma", f"{P.steer_model_sigma} rad/s", "EkfParams in fusion/"],
           ["Simulation tick", f"{P.dt * 1000:.0f} ms", "RK4; limit |λ|·dt < 2.785"],
           ["Relaxation length σ (option B)", "0.4 m", "assumed; spring rate C / σ"]],
          size=14)
add_notes(slide, "Reference slide: every figure on slides 2, 9, 11 and 12 follows from these values "
                 "(tools/slides/side_slip_model.py).")

prs.save(OUT)
print(f"wrote {OUT} ({len(prs.slides)} slides)")
for warning in warnings:
    print("WARNING:", warning)
