# Colibri Code Functions Wiki

This directory contains comprehensive wiki documentation for the Colibri Code Functions library.

## Wiki Pages

1. **[Home.md](Home.md)** - Wiki homepage with overview and navigation
2. **[Getting-Started.md](Getting-Started.md)** - Installation and setup guide
3. **[Drone-Module.md](Drone-Module.md)** - Complete drone flight control API
4. **[Camera-Module.md](Camera-Module.md)** - Camera and image processing
5. **[Servo-Module.md](Servo-Module.md)** - Servo motor control
6. **[Task-Framework.md](Task-Framework.md)** - Mission planning framework
7. **[Computer-Vision.md](Computer-Vision.md)** - CV capabilities (face detection, following, QR codes)
8. **[Examples-and-Tutorials.md](Examples-and-Tutorials.md)** - Practical examples and use cases
9. **[API-Reference.md](API-Reference.md)** - Complete API reference
10. **[Troubleshooting.md](Troubleshooting.md)** - Common issues and solutions
11. **[FAQ.md](FAQ.md)** - Frequently asked questions
12. **[Contributing.md](Contributing.md)** - Developer contribution guide

## Publishing to GitHub Wiki

GitHub wikis are stored in a separate git repository. To publish these pages:

### Option 1: Manual Upload (Recommended)

1. Go to your repository on GitHub
2. Click the "Wiki" tab
3. Click "Create the first page" or "New Page"
4. Copy content from each `.md` file and create corresponding wiki pages
5. Use the same filenames (without `.md` extension) for proper linking

### Option 2: Git Clone Method

```bash
# Clone the wiki repository
git clone https://github.com/EDUCA-DRONES/Colibri_CF.wiki.git

# Copy all markdown files
cp wiki/*.md Colibri_CF.wiki/

# Commit and push
cd Colibri_CF.wiki
git add .
git commit -m "Add comprehensive wiki documentation"
git push origin master
```

### Option 3: Use These Files Directly

These markdown files can also be:
- Included in the main repository `/docs` folder
- Used as GitHub Pages documentation
- Published to ReadTheDocs or similar platforms
- Distributed as PDF documentation

## Wiki Structure

```
Home (Home.md)
├── Getting Started (Getting-Started.md)
├── Core Modules
│   ├── Drone Module (Drone-Module.md)
│   ├── Camera Module (Camera-Module.md)
│   ├── Servo Module (Servo-Module.md)
│   └── Task Framework (Task-Framework.md)
├── Advanced Topics
│   ├── Computer Vision (Computer-Vision.md)
│   └── Examples and Tutorials (Examples-and-Tutorials.md)
└── Reference
    ├── API Reference (API-Reference.md)
    ├── Troubleshooting (Troubleshooting.md)
    ├── FAQ (FAQ.md)
    └── Contributing (Contributing.md)
```

## Documentation Statistics

- **Total Pages**: 12
- **Total Lines**: ~6,700
- **Total Words**: ~50,000
- **Coverage**: Complete API, examples, troubleshooting, and guides

## Features

### Comprehensive Coverage

✅ Installation and setup instructions  
✅ Complete API documentation for all modules  
✅ Practical examples and tutorials  
✅ Troubleshooting guides  
✅ FAQ section  
✅ Contributing guidelines  
✅ Code samples throughout  
✅ Navigation between pages  

### Topics Covered

- **Flight Control**: Navigation, waypoints, telemetry
- **Camera Operations**: Image capture, face detection, QR codes
- **Computer Vision**: Object tracking, person following
- **Servo Control**: Hardware control for gimbals and mechanisms
- **Mission Planning**: Task framework and autonomous missions
- **Safety**: Best practices and error handling
- **Development**: Contributing and coding standards

## Maintenance

To update the wiki:

1. Edit the `.md` files in this directory
2. Test locally with a markdown viewer
3. Commit changes to the repository
4. Publish updated files to GitHub Wiki

## License

This documentation is part of Colibri Code Functions and is licensed under the MIT License.

---

**Developed by**: Educa Drones Team  
**Repository**: https://github.com/EDUCA-DRONES/Colibri_CF  
**Version**: 1.1.4
