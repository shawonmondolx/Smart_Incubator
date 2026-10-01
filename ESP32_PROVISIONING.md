# ESP32 Wi-Fi Provisioning Contract

This document describes the firmware work required to connect the Flutter setup flow to the ESP32. The app currently uses `DemoProvisioningService`; it does not transmit Bluetooth data yet.

## Recommended Technology

Use Espressif's official provisioning stack rather than custom plain-text BLE characteristics:

- ESP32 transport: Bluetooth Low Energy
- ESP32 framework API: `WiFiProv`
- Security: Security 1 with proof of possession
- Credential storage: ESP32 NVS through the provisioning manager
- Device name: `INCUBATOR_PRO_<unique suffix>`
- Setup code: unique 8-12 character proof-of-possession value printed on the enclosure

Do not hardcode one shared setup code in production devices.

## Implemented Hobby Flow

The project now uses a fully free hobby flow with no Cloud Functions:

1. The user creates an email/password Firebase account in the app.
2. The app discovers `INCUBATOR_PRO_01` using Bluetooth LE.
3. The app sends the 2.4 GHz Wi-Fi SSID and password over the custom BLE service.
4. The ESP32 stores Wi-Fi credentials in NVS and connects to Firebase.
5. The user scans a QR label or enters `deviceId|claimCode` manually.
6. The app writes a claim request to Realtime Database.
7. The ESP32 verifies the claim code and assigns the authenticated user as owner.
8. Firebase rules allow that owner to read status and write desired settings.

This is suitable for a personal hobby project. The custom BLE payload is not encrypted at the application layer, the claim code is static, and the free claiming design is weaker than a trusted backend. Do not use this design for a commercial product.

Current prototype label content:

```text
incubator-01|K7PX-4M2Q-R9DT
```

Generate a QR code containing that exact text and attach it to the incubator. Change `DEVICE_CLAIM_CODE` in `include/secrets.h` for each future device.

BLE UUIDs:

```text
Service: 7f510001-6a7e-4bc4-9f2d-21a4d4671000
Command: 7f510002-6a7e-4bc4-9f2d-21a4d4671000
Status:  7f510003-6a7e-4bc4-9f2d-21a4d4671000
```

## ESP32 Behavior

1. On startup, read whether Wi-Fi credentials are provisioned.
2. If credentials exist, connect to Wi-Fi and start normal incubator operation.
3. If credentials do not exist, start BLE provisioning automatically.
4. Add a physical `SETUP` button. Holding it for five seconds should erase Wi-Fi credentials and restart provisioning.
5. Advertise a unique BLE name beginning with `INCUBATOR_PRO_`.
6. Require the printed proof-of-possession code.
7. Accept only Wi-Fi credentials through the secured provisioning session.
8. Send provisioning success or failure to the app.
9. Stop BLE provisioning after Wi-Fi connects to reduce attack surface and memory usage.
10. Continue all safety and incubation logic if Wi-Fi or cloud access fails.

## App Service Integration

Replace `DemoProvisioningService` in:

```text
app/lib/provisioning/provisioning_service.dart
```

The real implementation must satisfy these operations:

```text
scanDevices()
connect(device, proofOfPossession)
scanWifi()
provision(ssid, password)
disconnect()
```

Use a Flutter package compatible with ESP-IDF/Arduino ESP provisioning, or bridge Espressif's native Android and iOS provisioning libraries through Flutter platform channels. Confirm package maintenance and protocol compatibility before selecting it.

## Required Mobile Permissions

Android BLE and internet permissions are already declared in:

```text
app/android/app/src/main/AndroidManifest.xml
```

iOS Bluetooth and local-network descriptions are already declared in:

```text
app/ios/Runner/Info.plist
```

The final app must request runtime Bluetooth permissions before scanning.

## Local Access

After provisioning, the ESP32 should expose a local API and advertise itself using mDNS, for example:

```text
http://incubator-pro.local
```

Recommended local API transport:

- HTTPS when practical, otherwise authenticated HTTP only on the trusted LAN during prototyping
- WebSocket for live readings and equipment state
- REST endpoints for settings and commands
- A device identifier in every response

Do not rely permanently on a DHCP IP such as `192.168.1.125`; it can change.

## Firebase Remote Access

Wi-Fi provisioning alone works only while the phone and ESP32 are on the same network. This project will use Firebase for remote access:

```text
ESP32 -> HTTPS -> Firebase Realtime Database <- Flutter app
                              ^
                              |
                    Firebase Authentication
```

Firebase is responsible for telemetry synchronization, remote settings, commands, device ownership, and alerts. It must not run the incubator control loop. The ESP32 must continue regulating the incubator if Firebase or the internet is unavailable.

Never expose the ESP32 web server using router port forwarding.

## Firebase Signup and Cost

No account is needed for Bluetooth provisioning or local Wi-Fi control. Remote access requires one Google account and a Firebase project.

Start with the Firebase Spark plan:

- No monthly subscription is required.
- A payment card is normally not required for the Spark plan.
- The free quotas are suitable for development and a small personal installation.
- Usage and plan limits can change, so check the current Firebase pricing before production deployment.
- Do not enable paid Google Cloud services unless they are needed and billing limits are understood.

## Firebase Console Setup

1. Sign in at `https://console.firebase.google.com/` with a Google account.
2. Create a project named `smart-incubator` or another unique name.
3. Google Analytics is optional and is not required for this project.
4. Open **Build > Authentication** and enable Email/Password sign-in.
5. Open **Build > Realtime Database** and create a database in the region nearest to the users.
6. Do not leave the database in open test mode. Apply ownership-based rules before connecting a real device.
7. Register the Flutter Android app with package name `com.smartincubator.smart_incubator_app`.
8. Download `google-services.json` into `app/android/app/` when Flutter Firebase integration begins.
9. Register the iOS app if iPhone support is required and add the generated `GoogleService-Info.plist` through Xcode.
10. Use the FlutterFire CLI to generate `firebase_options.dart`; do not write Firebase configuration manually.

## Complete This Project's Firebase Setup

The remote-control code is implemented, but you must supply your Firebase project values. Do not send or download a service-account private key for this project.

### 1. Enable Authentication

In Firebase Console:

1. Open **Build > Authentication > Sign-in method**.
2. Enable **Email/Password** for the ESP32 prototype account.
3. Enable **Anonymous** for the current Flutter prototype. Replace anonymous app access with normal user sign-in before production.
4. Open **Authentication > Users** and add one device user, for example `device-01@your-domain.com`, with a strong unique password.
5. Copy the device user's Firebase UID. This is not a secret.

### 2. Configure Realtime Database

1. Open **Realtime Database** and copy its exact URL.
2. Open the **Rules** tab.
3. Paste the contents of `firebase.rules.json` and publish them.
4. Create this initial data, replacing both UID values:

```json
{
  "devices": {
    "incubator-01": {
      "deviceUid": "DEVICE_FIREBASE_UID",
      "ownerUid": "APP_USER_FIREBASE_UID"
    }
  }
}
```

For the current anonymous Flutter prototype, run the app once after Firebase configuration and retrieve its anonymous UID from **Authentication > Users**. Put that UID in `ownerUid`. Later, replace anonymous authentication with email/password user accounts.

### 3. Configure ESP32 Secrets

Edit the ignored local file:

```text
include/secrets.h
```

Set:

- `FIREBASE_API_KEY`: Firebase Console > Project settings > General > Web API Key
- `FIREBASE_DATABASE_URL`: exact Realtime Database URL
- `FIREBASE_DEVICE_EMAIL`: the dedicated device account email
- `FIREBASE_DEVICE_PASSWORD`: the dedicated device account password
- `FIREBASE_DEVICE_ID`: `incubator-01`, matching the database path

Do not use:

- A service-account JSON file
- A private key
- A legacy database secret
- Your personal Google password

### 4. Configure Flutter

Install the Firebase CLI and FlutterFire CLI, then authenticate and configure the app:

```powershell
npm install -g firebase-tools
firebase login
dart pub global activate flutterfire_cli
Set-Location "C:\Users\mayerdoya service\Documents\PlatformIO\Projects\Smart_Incubator\app"
flutterfire configure
```

Select your Firebase project and at least Android. The generated platform files allow `Firebase.initializeApp()` to connect. On Android, confirm this file exists afterward:

```text
app/android/app/google-services.json
```

The current app signs in anonymously and listens to:

```text
devices/incubator-01
```

It writes manual settings to:

```text
devices/incubator-01/desired
```

### 5. Upload and Verify

1. Close every serial monitor using COM5.
2. Upload the ESP32 firmware.
3. Open the serial monitor at 115200 baud.
4. Confirm Wi-Fi connection, Firebase token readiness, and no permission errors.
5. Open Realtime Database and confirm `status` and `telemetry` appear under `devices/incubator-01`.
6. Run the Flutter app and confirm the yellow demo-mode banner disappears.
7. Start a manual cycle and confirm `desired` is created.
8. Confirm the ESP32 writes the accepted values under `reported`.

Firebase client configuration files identify the project but are not administrator secrets. Database security must come from Authentication and Realtime Database rules.

## Database Structure

Use a unique, random device ID generated during manufacturing or first boot. Do not use the device IP address as its identity.

```text
users/
  {uid}/
    devices/
      {deviceId}/
        name: "Incubator 01"
        role: "owner"

devices/
  {deviceId}/
    ownerUid: "firebase-user-uid"
    status/
      online: true
      lastSeen: 1785354240000
      firmwareVersion: "0.1.0"
      localIp: "192.168.1.125"
    telemetry/
      temperature: 37.5
      humidity: 60.0
      heater: true
      fan: true
      humidifier: false
      eggTurner: false
      updatedAt: 1785354240000
    desired/
      targetTemperature: 37.5
      targetHumidity: 60.0
      automatic: true
      commandVersion: 12
    reported/
      targetTemperature: 37.5
      targetHumidity: 60.0
      automatic: true
      commandVersion: 12
    alerts/
      {alertId}/
        type: "HIGH_TEMPERATURE"
        active: true
        createdAt: 1785354240000
```

The app writes requested values under `desired`. The ESP32 validates them against hard safety limits, applies accepted values, and mirrors the actual configuration under `reported`. The UI must use `reported` to confirm that a command was accepted.

## Realtime Database Rules

The following is an initial ownership model. Test rules with the Firebase Emulator Suite before production use.

```json
{
  "rules": {
    ".read": false,
    ".write": false,
    "users": {
      "$uid": {
        ".read": "$uid === auth.uid",
        ".write": "$uid === auth.uid"
      }
    },
    "devices": {
      "$deviceId": {
        ".read": "auth != null && data.child('ownerUid').val() === auth.uid",
        "desired": {
          ".write": "auth != null && root.child('devices').child($deviceId).child('ownerUid').val() === auth.uid"
        }
      }
    }
  }
}
```

These rules cover authenticated app users only. They do not solve secure ESP32 identity. Do not put a Firebase service-account private key or unrestricted database secret in ESP32 firmware.

## ESP32 Authentication

For a personal prototype with one controlled device, an ESP32 Firebase client can authenticate with a dedicated Firebase email/password account that has access only to its own device path. Store credentials in NVS and avoid printing tokens or passwords to serial logs.

For production devices, use a backend enrollment flow:

1. Give each device a unique manufacturing secret or certificate.
2. The device proves its identity to a trusted backend endpoint.
3. The backend returns a short-lived Firebase custom token or narrowly scoped session.
4. The device refreshes credentials without embedding administrator secrets.
5. The backend links the device ID to the owner's Firebase UID during provisioning.

Firebase custom-token creation requires trusted server code and may require moving beyond a fully free architecture depending on hosting and usage. Implement the personal prototype first, then design production enrollment before selling or distributing devices.

## Flutter Firebase Work

When remote access implementation begins:

1. Install and run the FlutterFire CLI.
2. Add `firebase_core`, `firebase_auth`, and `firebase_database` to `app/pubspec.yaml`.
3. Initialize Firebase before `runApp`.
4. Add sign-up, sign-in, sign-out, password reset, and email verification screens.
5. Claim or link the provisioned `deviceId` to the authenticated user.
6. Listen to `devices/{deviceId}/telemetry`, `reported`, `status`, and `alerts` as realtime streams.
7. Write control changes only under `devices/{deviceId}/desired`.
8. Show commands as pending until `reported.commandVersion` matches the requested version.
9. Distinguish local, cloud, stale, and offline data in the UI.
10. Never place Firebase administrator credentials in the Flutter app.

## ESP32 Firebase Work

After Wi-Fi provisioning succeeds:

1. Synchronize time using NTP before starting authenticated TLS communication.
2. Authenticate using the selected prototype or production device identity method.
3. Publish telemetry at a controlled interval, such as every 5-15 seconds, rather than every sensor-loop iteration.
4. Update `status/lastSeen` and use a server-side or app-side timeout to determine offline state.
5. Listen or poll for changes under `desired`.
6. Validate temperature, humidity, mode, and actuator commands locally.
7. Reject unsafe values and report the actual accepted state under `reported`.
8. Buffer critical alerts if the internet is unavailable and send them after reconnection.
9. Reconnect using exponential backoff so network failure does not block the control loop.
10. Limit flash writes; do not save unchanged cloud values repeatedly.

## Data and Cost Control

- Send telemetry only when a value changes meaningfully or at a reasonable heartbeat interval.
- Keep one current telemetry object instead of continuously appending history.
- If historical charts are added, aggregate or limit samples and apply retention cleanup.
- Detach Flutter listeners when screens are closed.
- Monitor Realtime Database download, storage, and simultaneous-connection usage in the Firebase console.
- Configure budget alerts before upgrading to a billed plan.

## Safety Requirements

- Wi-Fi, Bluetooth, app, and cloud must never be required for temperature regulation.
- Validate every remote setpoint against safe minimum and maximum limits on the ESP32.
- Default all heater outputs off during boot, sensor failure, and invalid configuration.
- Add an independent thermostat and thermal fuse outside the ESP32 software.
- Store settings atomically and retain the last known safe configuration.
- Implement watchdog recovery and sensor timeout handling.
