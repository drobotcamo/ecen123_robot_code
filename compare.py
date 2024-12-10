import pandas as pd

def calculate_color_averages(file_path):
    """
    Reads a CSV file and calculates the average of each numeric column grouped by Color_Name.
    """
    # Read the CSV file
    df = pd.read_csv(file_path)

    # Ensure relevant columns are numeric
    numeric_columns = ['R_Value', 'G_Value', 'B_Value', 'C_Value', 'Color_Temp', 'Lux']
    df[numeric_columns] = df[numeric_columns].apply(pd.to_numeric, errors='coerce')

    # Group by Color_Name and calculate the average of each numeric column
    averages = df.groupby('Color_Name')[numeric_columns].mean()
    return averages

# File paths (replace with actual file paths)
file1 = 'colors.csv'
file2 = 'newcolors.csv'

# Calculate averages for each file
file1_averages = calculate_color_averages(file1)
file2_averages = calculate_color_averages(file2)

# Print averages for each file
print("Averages for File 1:")
print(file1_averages)
print("\nAverages for File 2:")
print(file2_averages)

# Compare averages
comparison = file1_averages.subtract(file2_averages, fill_value=0)
print("\nAverage Comparison per Column per Color (File 1 - File 2):")
print(comparison)
