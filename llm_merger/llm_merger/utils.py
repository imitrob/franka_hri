
from naive_merger.utils import cc

def clean_data(data):
    """Remove entries where either the label or timestamp is None."""
    if data is None:
        return []
    return [[float(timestamp), label] for timestamp, label in data if label is not None and timestamp is not None]
def normalize_timestamps(data, min_time):
    """Subtract the minimum timestamp to start from zero."""
    return [[float(timestamp) - float(min_time), label] for timestamp, label in data]


def print_modalities(text_data, gesture_data, timeline_width=80):
    # Clean data to remove None values
    text_data = clean_data(text_data)
    gesture_data = clean_data(gesture_data)

    # Find the minimum timestamp
    all_times = [ts for ts,_ in text_data] + [ts for ts,_ in gesture_data]
    if not all_times:
        print("No data to display.")
        return
    min_time = min(all_times)

    # Normalize timestamps
    text_data = normalize_timestamps(text_data, min_time)
    gesture_data = normalize_timestamps(gesture_data, min_time)

    # Find the new max time after normalization
    max_time = max(ts for ts,_ in text_data + gesture_data) if text_data or gesture_data else 1
    if max_time == 0.0: max_time = 0.1

    # Compute a scale factor to map time (in seconds) to character columns.
    scale = (timeline_width - 1) / max_time

    # Create header lines: one with tick marks and one with time labels.
    tick_line = [" "] * timeline_width
    label_line = [" "] * timeline_width

    # Let’s put ticks every (approximately) 10 columns (or adjust as needed).
    num_ticks = 8  # This will yield ticks at positions 0, ~10, ~20, ..., near timeline_width.
    tick_interval = timeline_width // num_ticks

    for i in range(num_ticks + 1):
        pos = i * tick_interval
        if pos >= timeline_width:
            pos = timeline_width - 1
        tick_line[pos] = "|"
        # Compute the corresponding time for this position.
        time_val = pos / scale
        time_str = f"{time_val:.2f}"
        # Center the time string at pos (if it fits)
        start = max(0, pos - len(time_str) // 2)
        for j, ch in enumerate(time_str):
            if start + j < timeline_width:
                label_line[start + j] = ch

    tick_line = "".join(tick_line)
    label_line = "".join(label_line)

    # Create empty lines for text and gestures.
    text_line = [" "] * timeline_width
    gesture_line = [" "] * timeline_width

    # Place each text event on the text_line.
    for timestamp,word in text_data:
        pos = int(timestamp * scale)
        if pos >= timeline_width:
            pos = timeline_width - 1
        for i, ch in enumerate(word):
            index = pos + i
            if index < timeline_width:
                text_line[index] = ch

    # Place each gesture event on the gesture_line.
    for timestamp,gesture in gesture_data:
        pos = int(timestamp * scale)
        if pos >= timeline_width:
            pos = timeline_width - 1
        for i, ch in enumerate(gesture):
            index = pos + i
            if index < timeline_width:
                gesture_line[index] = ch

    text_line = "".join(text_line)
    gesture_line = "".join(gesture_line)

    # Print the combined visualization.
    print(f"{cc.W}Time axis:{cc.E}")
    print(f"{cc.OKCYAN}"+label_line+f"{cc.E}")
    print(f"{cc.H}"+tick_line+f"{cc.E}")
    print(f"{cc.OKCYAN}" + text_line+f"{cc.E}")
    print(f"{cc.OKCYAN}" + gesture_line+f"{cc.E}")

if __name__ == "__main__":
    # Sample input: list of [timestamp, word]
    text_data = [
        [0.0, "pick"],
        [0.1, "up"],
        [0.2, "the"],
        [0.3, "cup"],
        [0.5, "and"],
        [0.6, "drink"]
    ]

    # Sample input: list of [timestamp, gesture]
    gesture_data = [
        [0.05, "wave"],
        [0.15, "point"],
        [0.3, "grab"],
        [0.75, "sip"] 
    ]

    print_modalities(text_data, gesture_data)