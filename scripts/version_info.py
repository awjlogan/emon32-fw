import subprocess


def extract_commit():
    try:
        commit = subprocess.run(
            ["git", "describe", "--always", "--tags", "--dirty"],
            capture_output=True,
            check=True,
            text=True,
        ).stdout.strip()
    except subprocess.CalledProcessError:
        commit = "None"

    return commit


def extract_version():
    found = 0
    with open("./src/emon32.h", "r") as f:
        for ln in f:
            if "VERSION_FW" in ln:
                var = ln.split()[-1].rstrip("u")
                if "MAJ" in ln:
                    maj = var
                    found += 1
                elif "MIN" in ln:
                    min = var
                    found += 1
                elif "REV" in ln:
                    rev = var
                    found += 1

            if found == 3:
                return (maj, min, rev)


def main():
    maj, min, rev = extract_version()
    commit = extract_commit()
    print(f"emon32-v{maj}.{min}.{rev}-{commit}")


if __name__ == "__main__":
    main()
