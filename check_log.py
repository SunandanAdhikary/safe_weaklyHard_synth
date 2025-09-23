def check_log(log_file_path = "d:\workspace\safe_weaklyHard_synth\models\suspension_control\log.txt"):
    # Read the log file and find the second non-blank line
    with open(log_file_path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    non_blank_lines = [line for line in lines if line.strip()]
    result = 0
    if len(non_blank_lines) >= 2:
        second_non_blank_line = non_blank_lines[1].strip()
        print(f"The second non-blank line is: {second_non_blank_line}")
        if "SAFE" in second_non_blank_line:
            print("Result: SAFE")
            result = 1
        elif "UNSAFE" in second_non_blank_line:
            print("Result: UNSAFE")
            result = 0
        elif "UNKNOWN" in second_non_blank_line:
            print("Result: UNKNOWN")
            result = -1
        else:
            print("The line does not contain SAFE, UNSAFE, or UNKNOWN. The computation might have been interrupted. Chweck the log file for details.")
            result = -2
    else:
        print("The file does not have a second non-blank line.")
        return
    return result
if __name__ == "__main__":
    check_log()

