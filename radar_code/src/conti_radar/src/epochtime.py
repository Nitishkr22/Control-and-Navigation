import time

# Function to convert epoch time in seconds to nanoseconds
def seconds_to_nanoseconds(epoch_time_seconds):
    epoch_time_nanoseconds = int(epoch_time_seconds * 1e9)
    return epoch_time_nanoseconds

# Example usage
epoch_time_seconds = time.time()  # Replace this with the epoch time in seconds you want to convert
epoch_time_nanoseconds = seconds_to_nanoseconds(epoch_time_seconds)

print(f"{epoch_time_seconds:.6f} seconds since epoch is equal to {epoch_time_nanoseconds:,} nanoseconds.")
