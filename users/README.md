# Kibo-RPC: How to Submit Source Code

This guide assumes that you are using WSL or Ubuntu.

## 1. Modify Your Source Code

Modify your source code in the user workspace.

Please follow these rules:

- Do not modify `krpc_api_server` or `krpc_ib2_msg`.
- Do not change the package names specified in `CMakeLists.txt` or `package.xml`.
- Do not rename the launch file.

## 2. Package Your Source Code

From the `users` directory, run `packing.sh` with your preferred archive name:

```bash
cd users
bash packing.sh my_submission
```

This command creates `my_submission.tar.gz`. You may use any name for the archive.

## 3. Log In to the Simulator Website

Open the following URL and log in:

https://jaxa.krpc.jp/user-auth

Use the account credentials provided by the event organizer.

## 4. Upload and Run Your Source Code

1. Click the **SIMULATION** tab.
2. Upload the generated archive to a **Slot**.
3. Click **START SIMULATION** to run the simulation.

Now, let's start developing!
