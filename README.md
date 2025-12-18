# Personal Portfolio | Interactive Engineering Showcase

A modern, responsive portfolio website showcasing projects in embedded systems, renewable energy optimization, and hardware-software integration. Built with performance and interactivity in mind.

**Live Demo:** [projects-page-xi.vercel.app](https://projects-page-xi.vercel.app)

## 🏗️ Technical Architecture

### **Frontend**
- **HTML5/CSS3/Vanilla JavaScript** - Zero framework dependencies for maximum performance
- **CSS Custom Properties** - Dynamic theming system (Tech/Eco modes)
- **Canvas API** - Particle constellation background with physics simulation
- **Web Audio API** - Real-time audio frequency/waveform analysis for vinyl visualizer
- **Three.js** - 3D globe rendering with progressive loading animation
- **Intersection Observer API** - Lazy-loaded scroll animations

### **Backend/API**
- **Vercel Serverless Functions** - OAuth token management and API proxying
- **Spotify Web API Integration** - Real-time playback status with refresh token flow
- **Environment-based secrets** - Secure credential storage

### **Deployment**
- **Vercel** - Edge network deployment with automatic HTTPS
- **Git-based CI/CD** - Automated builds on push to main

## 🎯 Key Features

### **1. Dual Theme System**
- **Tech Mode:** Cyan-purple gradient with electron constellation
- **Eco Mode:** Green gradient with leaf particles
- LocalStorage persistence across sessions

### **2. Live Spotify Integration**
```javascript
/api/now-playing → Spotify OAuth → Real-time track display
```
- Serverless function architecture prevents token exposure
- 5-second polling with client-side progress interpolation
- Album artwork, track metadata, and progress bar

### **3. Interactive Typing Test**
- Monkeytype-inspired UX with character-by-character validation
- Real-time WPM/accuracy calculation
- Speed graph with ghost line (previous attempt overlay)
- LocalStorage for performance history

### **4. Animated Background**
- 100-particle physics simulation
- Dynamic connection lines based on proximity (150px threshold)
- Spark effects at connection midpoints
- Optimized canvas rendering (60fps)

### **5. Audio Visualizer**
- **Web Audio API** - Real-time frequency and time domain analysis
- **FFT (Fast Fourier Transform)** - Spectrum analyzer display
- Dual visualization modes: Waveform (time domain) + Frequency spectrum
- Favorite vinyl album showcase integration
- Canvas-based rendering with gradient effects

### **6. Project Detail Pages**
- PCB-style circuit animations on hover (SVG path generation)
- Parallax 3D globe with Three.js
- Progressive triangle-by-triangle mesh construction
- Responsive image carousels

## 📁 Project Structure

```
Projects-Page/
├── index.html              # Main portfolio page
├── styles.css              # Global styles + theme system
├── projects/
│   ├── renewable-energy.html    # Project: Energy optimization
│   ├── piano.html               # Project: VCO synthesizer
│   └── race_me.html             # Interactive typing game
├── api/
│   └── now-playing.js           # Spotify serverless function
├── assets/                      # Images, media
└── get-spotify-token.py         # OAuth setup script
```

## 🔧 Setup & Development

### **Prerequisites**
- Node.js 18+ (for Vercel CLI)
- Spotify Developer Account (for Now Playing widget)

### **Local Development**
```bash
# Install Vercel CLI
npm i -g vercel

# Set environment variables
vercel env pull

# Run dev server
vercel dev
```

### **Spotify API Configuration**
1. Create app at [developer.spotify.com](https://developer.spotify.com/dashboard)
2. Run `python get-spotify-token.py` to obtain refresh token
3. Add credentials to Vercel environment:
   ```
   SPOTIFY_CLIENT_ID
   SPOTIFY_CLIENT_SECRET
   SPOTIFY_REFRESH_TOKEN
   ```

## 🎨 Design Decisions

### **Performance Optimizations**
- **No build step** - Direct HTML/CSS/JS for instant load
- **Lazy loading** - Intersection Observer for off-screen content
- **Canvas optimization** - RequestAnimationFrame with efficient draw calls
- **Asset preloading** - Critical resources loaded asynchronously

### **Accessibility**
- Semantic HTML5 structure
- ARIA labels for interactive elements
- Keyboard navigation support
- Focus states for all interactive components

## 🧪 Technical Highlights

**Particle System:**
- Velocity-based physics with edge collision detection
- Dynamic connection algorithm (O(n²) with distance optimization)
- Gradient spark effects using Canvas radial gradients

**Typing Test:**
- Keydown event handling with character validation
- Backspace support with state rollback
- WPM calculation: `(charIndex / 5) / (timeElapsed / 60)`
- Chart.js-free graph rendering with Canvas API

**3D Globe:**
- Three.js scene with perspective camera
- Custom shader for vertex coloring
- Multi-layer wireframe (3 concentric spheres)
- Mouse-influenced rotation quaternion

**Audio Visualizer:**
- Web Audio API with AnalyserNode for frequency data
- FFT size: 2048 samples for high-resolution spectrum
- Time domain: Oscilloscope-style waveform rendering
- Frequency domain: Bar-based spectrum analyzer (0-20kHz)
- Synchronized canvas animations at 60fps

## 🛠️ Technologies

| Category | Stack |
|----------|-------|
| Frontend | HTML5, CSS3, JavaScript (ES6+) |
| 3D Graphics | Three.js |
| API | Spotify Web API, Vercel Serverless |
| Hosting | Vercel (Edge Network) |
| Version Control | Git, GitHub |

## 📝 License

MIT License - Personal portfolio showcasing technical projects

---

**Built with:** ☕ + late nights at Caltech
