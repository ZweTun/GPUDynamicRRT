import csv
from argparse import ArgumentParser
from pathlib import Path

import yaml
from matplotlib import pyplot as plt
from PIL import Image


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("config_path")
    parser.add_argument("--waypoint_path", default=None)
    args = parser.parse_args()

    config_path = Path(args.config_path)
    waypoint_path = args.waypoint_path
    if waypoint_path is not None:
        waypoint_path = Path(waypoint_path)

    # Load config and image
    with config_path.open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    image_path = config_path.parent / config["image"]
    with Image.open(image_path) as image:
        image = image.convert("RGB")

    origin = config["origin"][0:2]
    resolution = config["resolution"]
    extent = (
        origin[0],
        origin[0] + image.width * resolution,
        origin[1],
        origin[1] + image.height * resolution,
    )

    fig, ax = plt.subplots()
    ax.imshow(image, extent=extent)

    # If existing waypoints are provided, plot them.
    if waypoint_path is not None:
        waypoints_x = []
        waypoints_y = []
        with waypoint_path.open(encoding="utf-8") as file:
            reader = csv.reader(file)
            for row in reader:
                waypoints_x.append(float(row[0]))
                waypoints_y.append(float(row[1]))

        ax.plot(
            waypoints_x,
            waypoints_y,
            marker="o",
            color="red",
            linewidth=1,
            markersize=3,
        )

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Map with Waypoints")

    def onclick(event):
        if event.inaxes != ax:
            return
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return
        # Print to console in "x,y" format.
        print("{:.3f},{:.3f}".format(x, y))
        # Plot a confirmation dot on the map.
        ax.plot(x, y, marker="o", color="blue", markersize=3)
        fig.canvas.draw_idle()

    # Connect the click event.
    fig.canvas.mpl_connect("button_press_event", onclick)

    plt.show()


if __name__ == "__main__":
    main()
