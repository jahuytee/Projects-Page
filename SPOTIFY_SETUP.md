# Spotify Now Playing - Deployment Guide

## ✅ What's Done
- ✅ Vercel serverless function created (`/api/now-playing.js`)
- ✅ Frontend widget updated to fetch real-time data
- ✅ Auth token obtained
- ✅ Code pushed to GitHub

## 🚀 Next Steps: Deploy to Vercel

### 1. Connect to Vercel

1. Go to [vercel.com](https://vercel.com) and sign up/login
2. Click "Add New" → "Project"
3. Import your GitHub repository: `jahuytee/Projects-Page`
4. Click "Import"

### 2. Add Environment Variables

Before deploying, add these **3 environment variables** in Vercel:

| Variable Name | Value |
|---------------|-------|
| `SPOTIFY_CLIENT_ID` | `e07738918f934e14ab1b3f9dc6cb8047` |
| `SPOTIFY_CLIENT_SECRET` | `a78332f7ae374d8aa28c0835152b7906` |
| `SPOTIFY_REFRESH_TOKEN` | `AQBlcDUll-w86B1VHHovNGQJe8hggIttubR8wkcCWE0PZqEG_OYyUFMGvlHvOlo4Eg4ShB2qXNzwfJ6b0ArkEOpXPGsTkSo1iLsJK7UBgHaWPmLjx7SPhE_7XyZZRz8lbOI` |

**How to add them:**
1. In Vercel project settings → "Environment Variables"
2. Add each variable (Name + Value)
3. Select "Production", "Preview", and "Development"
4. Click "Save"

### 3. Deploy

1. Click "Deploy" button
2. Wait ~1 minute for build to complete
3. Visit your site!

### 4. Test It

1. Play a song on Spotify
2. Wait 30 seconds (auto-refresh interval)
3. Your site should show the current track!

## 🔒 Security Check

✅ **Your tokens are safe:**
- Stored as Vercel environment variables (encrypted)
- Never exposed in browser
- Not in GitHub repo (`.gitignore` prevents this)
- API calls happen server-side only

## 🎵 How It Works

```
User's Browser → /api/now-playing (Vercel Function) → Spotify API
                      ↓
              Returns: {title, artist, albumArt}
```

- Function runs on Vercel's servers
- Uses refresh token to get access token
- Fetches currently playing track
- Returns public track info only

## 📝 Notes

- Widget updates every 30 seconds
- Shows "Not playing" when idle
- Click song title to open in Spotify
