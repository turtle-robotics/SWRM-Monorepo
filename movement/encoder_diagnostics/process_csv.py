import numpy as np
import matplotlib.pyplot as plt
import sys

# Find the difference between each value in the dataset
def get_differences(dataset, start, end):
    prev = dataset[0]
    transformation = [prev]

    for points in dataset[1:]:
        temp = points[:] # Copy to avoid direct borrowing
        for col in range(start, end):
            points[col] = points[col]-prev[col]
        transformation.append(points)
        prev = temp

    return transformation

# Use the first argument as the filename
def csv_to_array(filename):
    csv_source = open(filename, "r")
    print(f"Reading from {filename}...")

    # Write the data to csv_raw
    line = csv_source.readline()
    csv_raw = []
    while line:
        csv_raw += [line.split(",")]
        line = csv_source.readline().strip()

    # Convert numbers to integers
    for i in range(len(csv_raw)):
        for j in range(len(csv_raw[i])):
            try:
                csv_raw[i][j] = int(csv_raw[i][j])
            except ValueError:
                csv_raw[i][j] = csv_raw[i][j]

    return csv_raw

# Begin main code
def main():
    filenames = sys.argv[1:]
    datasets = []
    dataset_lines = []
    dataset_diffs = []

    # Process files the user specified as raw encoder data and differential data
    for file in filenames:
        datasets.append(csv_to_array(file))
        dataset_lines.append(np.array(datasets[-1][1:]).reshape(len(datasets[-1])-1, len(datasets[-1][0])))
        dataset_diffs.append(get_differences(datasets[-1][1:], 1, 4))
        dataset_diffs[-1] = np.array(dataset_diffs[-1]).reshape(len(dataset_diffs[-1]), len(dataset_diffs[-1][0]))
 
    # Begin plotting data
    for i in range(len(dataset_diffs)):
        figure, (line, diffs) = plt.subplots(2)
        plt.figure(i+1)

        # Plot the raw encoder data
        figure.suptitle(sys.argv[i+1])
        line.plot(dataset_lines[i][:, 1], "bo", label="Motor 1")
        line.plot(np.abs(dataset_lines[i][:, 2]), "ro", label="Motor 2")
        line.plot(dataset_lines[i][:, 3], "go", label="Motor 3")
        line.set_ylabel("Raw encoder counts")
        line.set_xlabel("Iterations")
        line.legend(loc='lower right')

        # Plot the encoder count differences
        diffs.plot(dataset_diffs[i][:, 1], "bo", label="Motor 1")
        diffs.plot(np.abs(dataset_diffs[i][:, 2]), "ro", label="Motor 2")
        diffs.plot(dataset_diffs[i][:, 3], "go", label="Motor 3")
        diffs.set_ylabel("Differences in encoder counts")
        diffs.set_xlabel("Iterations")
        diffs.legend(loc='lower right')
        diffs.set_ylim(2750, 3400)

    # Show plot
    plt.show()
    plt.close()

if __name__ == "__main__":
    main()
