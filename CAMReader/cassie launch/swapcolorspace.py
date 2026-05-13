# “””
# Generate a contact sheet of YUYV/UYVY color recovery attempts.

# Usage:
# python3 yuyv_contact_sheet.py <input_image> [output_contact_sheet.jpg]

# Produces a contact sheet comparing 6 recovery methods side-by-side with the original.
# “””

import sys
import numpy as np
from PIL import Image


def generate_contact_sheet(input_path: str, output_path: str = "contact_sheet.jpg"):
    orig = Image.open(input_path).convert("RGB")
    orig_np = np.array(orig, dtype=np.float32)

    img_ycbcr = np.array(orig.convert("YCbCr"), dtype=np.float32)
    Y_c = img_ycbcr[:, :, 0]
    Cb_c = img_ycbcr[:, :, 1]
    Cr_c = img_ycbcr[:, :, 2]

    results = {}

    # Method A: Grayscale recovery — true luma from average of Cb+Cr, neutral chroma
    Y_true = (Cb_c + Cr_c) / 2
    Cb_true = np.full_like(Y_true, 128.0)
    Cr_true = np.full_like(Y_true, 128.0)
    arr = np.stack([Y_true, Cb_true, Cr_true], axis=2).clip(0, 255).astype(np.uint8)
    results["A: Grayscale recovery"] = Image.fromarray(arr, "YCbCr").convert("RGB")

    # Method B: Use Cb channel as Y, Y as chroma hint
    Y_true = Cb_c
    Cb_true = np.clip(128 + (Y_c - 128) * 1.5, 0, 255)
    Cr_true = np.clip(128 + (Cr_c - 128) * 0.5, 0, 255)
    arr = np.stack([Y_true, Cb_true, Cr_true], axis=2).clip(0, 255).astype(np.uint8)
    results["B: Cb as Y"] = Image.fromarray(arr, "YCbCr").convert("RGB")

    # Method C: Use Cr channel as Y, Y as chroma hint
    Y_true = Cr_c
    Cb_true = np.clip(128 + (Y_c - 128) * 1.5, 0, 255)
    Cr_true = np.clip(128 + (Cb_c - 128) * 0.5, 0, 255)
    arr = np.stack([Y_true, Cb_true, Cr_true], axis=2).clip(0, 255).astype(np.uint8)
    results["C: Cr as Y"] = Image.fromarray(arr, "YCbCr").convert("RGB")

    # Method D: Simple RGB channel swap R <-> G
    t = orig_np.copy()
    t[:, :, 0], t[:, :, 1] = orig_np[:, :, 1].copy(), orig_np[:, :, 0].copy()
    results["D: Swap R<->G"] = Image.fromarray(t.astype(np.uint8))

    # Method E: Proper UYVY->YUYV reversal
    # YUYV bytes [Y0,U,Y1,V] misread as UYVY [U,Y0,V,Y1]:
    #   decoded Y  = real U (Cb)  → true Y  = Cb_corrupted
    #   decoded Cb = real Y even  → true Cb = derived from Y_corrupted
    #   decoded Cr = real Y odd   → true Cr = Cr_corrupted remapped
    Y_true = np.clip(Cb_c, 0, 255)
    Cb_true = np.clip(128 + (Y_c - Y_true.mean()) * 0.8, 0, 255)
    Cr_true = np.clip(128 + (Cr_c - 128) * 0.3, 0, 255)
    arr = np.stack([Y_true, Cb_true, Cr_true], axis=2).clip(0, 255).astype(np.uint8)
    results["E: Proper UYVY fix"] = Image.fromarray(arr, "YCbCr").convert("RGB")

    # Method F: Histogram-stretched luma + chroma hint from corrupted Y channel
    Y_true = (Cb_c + Cr_c) / 2
    Y_min, Y_max = Y_true.min(), Y_true.max()
    Y_stretched = (Y_true - Y_min) / (Y_max - Y_min) * 255
    chroma_hint = Y_c - Y_c.mean()
    Cb_true = np.clip(128 - chroma_hint * 0.6, 0, 255)
    Cr_true = np.clip(128 + chroma_hint * 0.2, 0, 255)
    arr = (
        np.stack([Y_stretched, Cb_true, Cr_true], axis=2).clip(0, 255).astype(np.uint8)
    )
    results["F: Stretched + chroma"] = Image.fromarray(arr, "YCbCr").convert("RGB")

    # --- Build contact sheet ---
    all_imgs = [("Original", orig)] + list(results.items())
    n = len(all_imgs)
    cols = 3
    rows = -(-n // cols)  # ceiling division

    w, h = orig.size
    label_h = 28
    thumb_w, thumb_h = w, h

    sheet = Image.new("RGB", (thumb_w * cols, (thumb_h + label_h) * rows), (30, 30, 30))

    try:
        from PIL import ImageDraw, ImageFont

        draw = ImageDraw.Draw(sheet)
        font = ImageFont.load_default()
    except Exception:
        draw = None

    for i, (label, img) in enumerate(all_imgs):
        col = i % cols
        row = i // cols
        x = col * thumb_w
        y = row * (thumb_h + label_h)
        sheet.paste(img.resize((thumb_w, thumb_h)), (x, y + label_h))
        if draw:
            draw.rectangle([x, y, x + thumb_w, y + label_h], fill=(50, 50, 50))
            draw.text((x + 6, y + 6), label, fill=(220, 220, 220), font=font)

        r = np.array(img, dtype=np.float32)
        print(
            f"{label:30s}  R={r[:,:,0].mean():.1f}  G={r[:,:,1].mean():.1f}  B={r[:,:,2].mean():.1f}"
        )

    # Scale down if very wide
    max_w = 2400
    if sheet.width > max_w:
        scale = max_w / sheet.width
        sheet = sheet.resize((int(sheet.width * scale), int(sheet.height * scale)))

    sheet.save(output_path, quality=92)
    print(f"\nContact sheet saved to: {output_path}")


input_file = sys.argv[1] if len(sys.argv) > 1 else "input.jpg"
output_file = sys.argv[2] if len(sys.argv) > 2 else "contact_sheet.jpg"
generate_contact_sheet(input_file, output_file)
