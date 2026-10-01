#pragma once

// Copy this file to include/secrets.h and replace every placeholder.
// Never use a Firebase service-account key or legacy database secret here.
constexpr char WIFI_SSID[] = "YOUR_2_4_GHZ_WIFI";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

// Firebase Console > Project settings > General > Web API Key.
constexpr char FIREBASE_API_KEY[] = "YOUR_FIREBASE_WEB_API_KEY";

// Use the exact Realtime Database URL shown in Firebase Console.
// Example: https://smart-incubator-default-rtdb.firebaseio.com
constexpr char FIREBASE_DATABASE_URL[] = "https://YOUR_PROJECT-default-rtdb.firebaseio.com";

// Dedicated Firebase Authentication email/password account for this prototype
// device. Restrict it to this device path with database rules.
constexpr char FIREBASE_DEVICE_EMAIL[] = "device-01@example.com";
constexpr char FIREBASE_DEVICE_PASSWORD[] = "YOUR_DEVICE_ACCOUNT_PASSWORD";

constexpr char FIREBASE_DEVICE_ID[] = "incubator-01";

// Print this device ID and claim code as a QR label. Use a different random
// claim code for every physical device.
constexpr char DEVICE_CLAIM_CODE[] = "K7PX-4M2Q-R9DT";
