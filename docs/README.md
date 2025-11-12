# Documentation

Comprehensive documentation for the zkML Robotics Demo system.

## 📚 Quick Navigation

### Getting Started
- **[QUICKSTART.md](QUICKSTART.md)** - Fast setup guide to get the demo running
- **[README.md](../README.md)** - Main project overview (in repository root)

### Setup Guides
- **[CAMERA_SCRIPTS.md](CAMERA_SCRIPTS.md)** - Multi-platform camera setup (Linux, WSL, Windows)
- **[WINDOWS_CAMERA_SETUP.md](WINDOWS_CAMERA_SETUP.md)** - Detailed Windows camera configuration
- **[CAMERA_ID_1_INSTRUCTIONS.md](CAMERA_ID_1_INSTRUCTIONS.md)** - Camera device selection

### Architecture & Implementation
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - System architecture and component design
- **[COMMITMENT_BINDING_IMPLEMENTATION.md](COMMITMENT_BINDING_IMPLEMENTATION.md)** - Cryptographic proof binding details
- **[SECURITY_FIXES.md](SECURITY_FIXES.md)** - Security improvements and fixes applied

### Development
- **[TESTING.md](TESTING.md)** - Testing procedures and verification
- **[CLAUDE.md](CLAUDE.md)** - Development log and recent improvements
- **[UI_ENHANCEMENTS.md](UI_ENHANCEMENTS.md)** - Web UI improvements and fixes
- **[SCRIPTS.md](SCRIPTS.md)** - Root directory scripts documentation

### Business
- **[INVESTOR_BRIEF.md](INVESTOR_BRIEF.md)** - Project overview for investors

## 📖 Documentation by Topic

### For First-Time Users
1. Read [../README.md](../README.md) - Project overview
2. Follow [QUICKSTART.md](QUICKSTART.md) - Get demo running
3. Check [CAMERA_SCRIPTS.md](CAMERA_SCRIPTS.md) - Set up your camera

### For Developers
1. Review [ARCHITECTURE.md](ARCHITECTURE.md) - Understand the system
2. Read [COMMITMENT_BINDING_IMPLEMENTATION.md](COMMITMENT_BINDING_IMPLEMENTATION.md) - Learn about zkML proofs
3. Check [SECURITY_FIXES.md](SECURITY_FIXES.md) - Security best practices
4. See [TESTING.md](TESTING.md) - Test procedures
5. Review [CLAUDE.md](CLAUDE.md) - Recent development history

### For Deployers
1. Follow [QUICKSTART.md](QUICKSTART.md) - Initial setup
2. Review [SCRIPTS.md](SCRIPTS.md) - Understand control scripts
3. Check camera guides for your platform
4. Review [SECURITY_FIXES.md](SECURITY_FIXES.md) - Security considerations

## 🔍 Quick Reference

### Key Concepts

**zkML (Zero-Knowledge Machine Learning)**
- Cryptographic proofs verify ML inference without revealing model weights
- JOLT-Atlas generates proofs in 3-4 seconds
- Proofs are portable and can be verified independently

**System Components**
- **zkml_guard**: ROS 2 node managing inference and proof generation
- **onnx-verifier**: HTTP service generating JOLT proofs
- **robotics-ui**: Web interface for demos and control
- **twist_mux**: Robot motion multiplexer with proof-gated safety

**File Locations**
- Source code: `src/zkml_guard/`
- Web UI: `tools/robotics-ui/`
- Verifier: `tools/onnx-verifier/`
- Scripts: Root directory and `scripts/`
- Documentation: `docs/` (this directory)

### Common Tasks

**Start the demo:**
```bash
./start_demo.sh
```

**Stop the demo:**
```bash
./stop_demo.sh
```

**Access web UI:**
```
http://localhost:9200
```

**Check logs:**
```bash
ls /tmp/rdemo-*.log
```

## 📝 Document Index

| Document | Topic | Audience |
|----------|-------|----------|
| [QUICKSTART.md](QUICKSTART.md) | Fast setup guide | Everyone |
| [ARCHITECTURE.md](ARCHITECTURE.md) | System design | Developers |
| [COMMITMENT_BINDING_IMPLEMENTATION.md](COMMITMENT_BINDING_IMPLEMENTATION.md) | Cryptographic proofs | Developers/Security |
| [SECURITY_FIXES.md](SECURITY_FIXES.md) | Security improvements | Developers/Security |
| [TESTING.md](TESTING.md) | Test procedures | Developers/QA |
| [CLAUDE.md](CLAUDE.md) | Development log | Developers |
| [UI_ENHANCEMENTS.md](UI_ENHANCEMENTS.md) | UI improvements | Developers/UI |
| [CAMERA_SCRIPTS.md](CAMERA_SCRIPTS.md) | Camera setup | Everyone |
| [WINDOWS_CAMERA_SETUP.md](WINDOWS_CAMERA_SETUP.md) | Windows camera | Windows users |
| [CAMERA_ID_1_INSTRUCTIONS.md](CAMERA_ID_1_INSTRUCTIONS.md) | Device selection | Troubleshooting |
| [SCRIPTS.md](SCRIPTS.md) | Script documentation | Everyone |
| [INVESTOR_BRIEF.md](INVESTOR_BRIEF.md) | Business overview | Investors |

## 🤝 Contributing

When adding new documentation:
1. Place in this `docs/` directory
2. Update this README with link and description
3. Follow existing formatting style
4. Include code examples where appropriate
5. Test all command examples before committing

## 📧 Support

For issues or questions:
- Check relevant documentation above
- Review [TESTING.md](TESTING.md) for troubleshooting
- Check GitHub issues: https://github.com/hshadab/robotics/issues

---

**Note**: This is a prototype demonstration system using production-grade zkML cryptography (JOLT-Atlas) but with localhost-only deployment architecture.
