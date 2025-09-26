from PyPDF2 import PdfReader, PdfWriter
from reportlab.pdfgen import canvas
import io
import itertools as it

BUILDINGS = ["EXP", "ISEC"]
EXP_LOCATIONS = ["doors near Ruggles", "elevators", "doors near ISEC"]
EXP_FLOORS = [1, 2, 3, 4, 5, 6, 7]
EXP_OPTIONS = []
for floor, location in it.product(EXP_FLOORS, EXP_LOCATIONS):
    EXP_OPTIONS.append(f"EXP Floor {floor}, {location}")

print(EXP_OPTIONS)

ISEC_LOCATIONS = ["near train", "Elevators", "near Columbus Ave"]
ISEC_FLOORS = [1, 2, 3, 4, 5, 6, "Basement"]
ISEC_OPTIONS = []
for floor, location in it.product(ISEC_FLOORS, ISEC_LOCATIONS):
    if floor == "Basement":
        ISEC_OPTIONS.append(f"ISEC {floor}, {location}")
    else:
        ISEC_OPTIONS.append(f"ISEC Floor {floor}, {location}")
print(ISEC_OPTIONS)

TAG_LABELS = EXP_OPTIONS + ISEC_OPTIONS
print(f"Total options: {len(TAG_LABELS)}")

def add_border_messages_to_pdf(
    input_pdf,
    output_pdf,
    top_message=None,
    bottom_message=None,
    left_message=None,
    right_message=None,
    *,
    top_margin=24,
    bottom_margin=24,
    left_margin=24,
    right_margin=24,
    font="Helvetica-Bold",
    font_size=12
):
    """
    Add text messages around all four borders of each PDF page.

    - Top/Bottom text is drawn horizontally and centered.
    - Left/Right text is drawn vertically and centered along the page height.
      Left reads bottom->top; Right reads top->bottom (standard book spine behavior).

    Arguments:
        input_pdf:   path to source PDF
        output_pdf:  path to destination PDF
        *_message:   strings to draw on each border (use None to skip)
        *_margin:    distance from corresponding border (in points; 72 pt = 1 inch)
        font:        reportlab font name (e.g., 'Helvetica', 'Helvetica-Bold')
        font_size:   font size in points
    """
    reader = PdfReader(input_pdf)
    writer = PdfWriter()

    cnt = 0
    for page in reader.pages:
        width = float(page.mediabox.width)
        height = float(page.mediabox.height)

        # Build an overlay the exact size of this page
        packet = io.BytesIO()
        can = canvas.Canvas(packet, pagesize=(width, height))
        can.setFont(font, font_size)

        # --- Top (horizontal, centered) ---
        if top_message:
            y_top = max(top_margin, min(height - top_margin, height - top_margin))
            can.drawCentredString(width / 2.0, y_top, top_message)

        # --- Bottom (horizontal, centered) ---
        bottom_message = TAG_LABELS[cnt] if cnt < len(TAG_LABELS) else None
        cnt += 1
        if bottom_message:
            y_bot = max(bottom_margin, min(height - bottom_margin, bottom_margin))
            can.drawCentredString(width / 2.0, y_bot, bottom_message)

        # --- Left (vertical, centered along height), read bottom -> top ---
        if left_message:
            can.saveState()
            # Move to left margin at vertical center, then rotate CCW 90°
            can.translate(left_margin, height / 2.0)
            can.rotate(90)
            # After rotation, x-axis points up; draw centered at (0,0)
            can.drawCentredString(0, 0, left_message)
            can.restoreState()

        # --- Right (vertical, centered along height), read top -> bottom ---
        if right_message:
            can.saveState()
            # Move to right margin, vertical center; rotate CW 90°
            can.translate(width - right_margin, height / 2.0)
            can.rotate(-90)
            can.drawCentredString(0, 0, right_message)
            can.restoreState()

        can.save()
        packet.seek(0)

        overlay_pdf = PdfReader(packet)
        overlay_page = overlay_pdf.pages[0]

        # Merge overlay text onto the original page
        page.merge_page(overlay_page)
        writer.add_page(page)

    with open(output_pdf, "wb") as f:
        writer.write(f)

    print(f"Messages added to: {output_pdf}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Add messages to all four borders of each PDF page."
    )
    parser.add_argument("input_pdf", help="Path to the input PDF file.")
    parser.add_argument("output_pdf", help="Path to the output PDF file.")
    parser.add_argument("--top_message", default="APPROVED BY BUILDINGS OPERATIONS DIRECTOR", help="Bottom border text.")
    parser.add_argument("--bottom_message", default="DO NOT REMOVE", help="Top border text.")
    parser.add_argument("--left_message", default="ONGOING EXPERIMENT TODAY. WILL BE REMOVED PROMPTLY", help="Left border text (vertical).")
    parser.add_argument("--right_message", default="DO NOT REMOVE! REFER ANY QUESTIONS TO ISEC/EXP OPERATIONS DIRECTOR", help="Right border text (vertical).")

    side_margin = 96
    parser.add_argument("--top_margin", type=float, default=36, help="Top margin in points.")
    parser.add_argument("--bottom_margin", type=float, default=24, help="Bottom margin in points.")
    parser.add_argument("--left_margin", type=float, default=side_margin, help="Left margin in points.")
    parser.add_argument("--right_margin", type=float, default=side_margin, help="Right margin in points.")

    parser.add_argument("--font", default="Helvetica-Bold", help="ReportLab font name.")
    parser.add_argument("--font_size", type=float, default=24, help="Font size in points.")

    args = parser.parse_args()

    add_border_messages_to_pdf(
        args.input_pdf,
        args.output_pdf,
        top_message=args.top_message,
        bottom_message=args.bottom_message,
        left_message=args.left_message,
        right_message=args.right_message,
        top_margin=args.top_margin,
        bottom_margin=args.bottom_margin,
        left_margin=args.left_margin,
        right_margin=args.right_margin,
        font=args.font,
        font_size=args.font_size,
    )
