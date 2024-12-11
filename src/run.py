
import pandas as pd
import subprocess

with open("messages.csv", "r") as messages_file:
    df = pd.read_csv(messages_file, dtype={"src_hw": str, "dest_hw": str,
                                           "src_neuron": str})

print(df)
cycle_times = []
for timestep in range(1, 129):
    filtered_df = df[df.iloc[:, 0] == timestep]
    with open(f"messages_single_ts.csv", "w") as output_file:
        filtered_df.to_csv(output_file, index=False)
    result = subprocess.run(("./booksim", "examples/spike"),
                            capture_output=True, text=True)

    for line in result.stdout.split('\n'):
        if "Time taken is" in line:
            # Extract the number of cycles
            cycles = int(line.split("is")[1].split()[0])
            cycle_times.append(cycles)
            print(f"Timestep {timestep}: {cycles} cycles")
            break

assert(len(cycle_times) == 128)
print(cycle_times)