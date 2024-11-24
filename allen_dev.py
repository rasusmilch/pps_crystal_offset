import argparse
import re
import matplotlib.pyplot as plt
import allantools
import os

def parse_log_file(filename):
    """Parse the log file to extract tick differences."""
    tick_differences = []
    with open(filename, 'r') as file:
        for line in file:
            # Match lines with "Difference:" and extract the value
            match = re.search(r"Difference:\s+(\d+)", line)
            if match:
                # print(int(match.group(1)))
                tick_differences.append(int(match.group(1)))
    return tick_differences

def calculate_allan_deviation(tick_differences):
    """Calculate Allan deviation using the mean of tick differences as nominal."""
    # Compute the nominal value as the mean of all tick differences
    nominal_ticks = sum(tick_differences) / len(tick_differences)
    fractional_frequencies = [(ticks - nominal_ticks) / nominal_ticks for ticks in tick_differences]

    # Calculate Allan deviation using allantools
    taus, adev, _, _ = allantools.oadev(fractional_frequencies, rate=1, data_type="freq")
    return taus, adev

def plot_allan_deviation(taus, adev, output_file):
    """Plot and save the Allan deviation graph."""
    plt.figure(figsize=(12, 8))  # Set figure size (width=12 inches, height=8 inches)
    plt.loglog(taus, adev)
    plt.xlabel("Averaging Time (τ) [s]", fontsize=14)
    plt.ylabel("Allan Deviation σy(τ)", fontsize=14)
    plt.title("Allan Deviation Plot", fontsize=16)
    plt.grid(True, which="both", linestyle="--", linewidth=0.5)
    plt.tight_layout()  # Adjust layout to prevent overlap
    plt.savefig(output_file, dpi=300)  # Save with high resolution
    plt.show()
    print(f"Allan deviation plot saved to {output_file}")

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Parse log file and generate Allan deviation plot.")
    parser.add_argument("logfile", help="Path to the log file containing tick differences")
    # parser.add_argument("-o", "--output", default="allan_deviation.png", help="Output filename for the plot")
    args = parser.parse_args()

    # Parse the log file
    print(f"Parsing log file: {args.logfile}")
    tick_differences = parse_log_file(args.logfile)
    if not tick_differences:
        print("No tick differences found in the log file. Exiting.")
        return

    # Calculate Allan deviation
    print("Calculating Allan deviation...")
    taus, adev = calculate_allan_deviation(tick_differences)

    # Generate output file name by replacing the extension
    base_name, _ = os.path.splitext(args.logfile)
    output_file = f"{base_name}.png"

    # Plot and save the Allan deviation
    print("Generating plot...")
    plot_allan_deviation(taus, adev, output_file)

if __name__ == "__main__":
    main()
