#!/usr/bin/env python3
import sys
from typing import Literal
from typing_extensions import assert_never
import ats_playback
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.pyplot as pp
import numpy as np
import PIL.Image

cfg, packets = ats_playback.read_file(sys.argv[1])

OBJ_ID = 1
SENSOR: Literal["wf", "nf"] = "wf"
MODE: Literal["heatmap", "animation"] = "animation"

arr = []
screen_ids = []

for timestamp, pkt in packets:
    if isinstance(pkt.data, pkt.data.ObjectReport):
        obj_report = pkt.data._0
        if SENSOR == "wf":
            arr.append(obj_report.mot_data_wf[OBJ_ID].area)
        elif SENSOR == "nf":
            arr.append(obj_report.mot_data_nf[OBJ_ID].area)
        else:
            assert_never(SENSOR)
    elif isinstance(pkt.data, pkt.data.CombinedMarkersReport):
        report = pkt.data._0
        if SENSOR == "wf":
            screen_ids.append(report.wf_screen_ids[OBJ_ID])
        elif SENSOR == "nf":
            screen_ids.append(report.nf_screen_ids[OBJ_ID])
        else:
            assert_never(SENSOR)

print(f"{len(arr) = }")
print(f"{len(screen_ids) = }")

mat = []
peaks = []
compressed_screen_ids = []
for i in range(0, len(arr)//128*128, 128):
    mat.append(abs(np.fft.rfft(arr[i:i+128])[1:]))
    peaks.append(ats_playback.peak_detect_128(arr[i:i+128]))
    compressed_screen_ids.append(min(screen_ids[i:i+128]))

print(f"{peaks = }")
print(f"{compressed_screen_ids = }")

freq = np.fft.rfftfreq(128, 1/200)[1:]

if MODE == "heatmap":
    pp.imshow(mat, vmax=100, extent=(200/128, 100, 0, 100))
    pp.title(f"{SENSOR.upper()} Object {OBJ_ID}")
    pp.show()
elif MODE == "animation":
    ims = []
    for i, (row, peak) in enumerate(zip(mat, peaks)):
        fig = pp.figure()
        canvas = FigureCanvasAgg(fig)
        ax = fig.add_subplot()
        ax.set_ylim(0, 50)
        ax.plot(freq, row)
        ax.axvline(peak, c='r')
        ax.set_title(f"Frame {i}; peak={peak:06.3f}")
        ax.set_xlabel("Frequency")
        fig.canvas.draw()
        im = PIL.Image.frombytes("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba())
        ims.append(im)
        pp.close(fig)

    ims[0].save("out.gif", save_all=True, append_images=ims[1:], duration=100)
else:
    assert_never(MODE)
