# Unreal Engine setup for Claude Code (MCP)

Goal: Claude Code can inspect and change the Unreal Editor scene directly, for example spawn the arm, move joints and read actor transforms.
This belongs to the Unreal option in `simulator-proposal.md`.

Status on 2026-10-12: Unreal Engine 5.8 is being installed on the team machine.
Claude Code checked the rest of the toolchain on the same machine; the results are in section 2.

## 1. Version conflict: UE 5.8 vs. URLab

| | UE 5.7 | UE 5.8 |
|-|--------|--------|
| Epic's built-in Unreal MCP plugin | No | Yes, experimental |
| URLab (MuJoCo inside Unreal) | Yes, the only tested version | Not supported. The README warns about Vulkan driver regressions on Linux, so Windows may work, untested |
| Community MCP servers (Python remote execution) | Yes | Probably, untested |

Suggestion: use **UE 5.8 with Epic's MCP** to get Claude Code working with the editor now.
If we go for URLab later, install UE 5.7 next to it (the Epic launcher can hold several versions) or try URLab on 5.8 on Windows.

## 2. Toolchain on this machine

| Tool | Needed for | State on 2026-10-12 |
|------|------------|---------------------|
| Unreal Engine 5.8 | Everything | Installing, in `C:\Program Files\Epic Games\UE_5.8` |
| Visual Studio 2022 with "Game development with C++" | C++ projects, URLab, C++ plugins | **Not installed** |
| CMake 3.24+ | Building URLab's dependencies (MuJoCo, CoACD, libzmq) | **Not installed** |
| Python 3.11+ | URLab's `urlab_bridge`, our own scripts | Installed (3.13 and 3.15) |
| Node.js 18+ | Only for the community MCP server `sam-david/unreal-mcp` | Not installed, not needed for Epic's MCP |
| Git | Cloning plugins | Installed |

Epic's MCP plugin is built into the engine, so it works in a Blueprint-only project without Visual Studio.
Visual Studio and CMake only matter once we add URLab or our own C++ code.

## 3. Epic's Unreal MCP (UE 5.8)

Source: Epic's documentation page "Unreal MCP in Unreal Editor".

1. Create the project. Choose **C++** instead of Blueprint if URLab may follow, because URLab only works in C++ projects. A C++ project needs Visual Studio.
2. **Edit > Plugins**: enable **Unreal MCP** and **All Toolsets**. The Toolset Registry plugin is enabled automatically. Restart the editor.
3. **Editor Preferences > General > Model Context Protocol**: turn on **Auto Start Server**. The defaults are port `8000` and path `/mcp`, so the server runs at `http://127.0.0.1:8000/mcp`.
4. Or start it manually in the editor console: `ModelContextProtocol.StartServer`.

Shipped toolsets include `SceneTools`, `ActorTools`, `MaterialInstanceTools` and `ObjectTools`.
Community guides report that the toolsets sometimes have to be enabled by hand before any tools appear.
After changing toolsets, run `ModelContextProtocol.RefreshTools`.

### Connecting Claude Code

This repo already has a project-scoped `.mcp.json` pointing at the editor:

```json
{ "mcpServers": { "unreal": { "type": "http", "url": "http://127.0.0.1:8000/mcp" } } }
```

So the steps are:

1. Open the Unreal project and make sure the MCP server is running.
2. Start Claude Code in this repo. Approve the `unreal` server when Claude Code asks about it.
3. Check the connection with `claude mcp list` or `/mcp`. While the editor is closed, `unreal` shows as failed; that is expected.

The editor can also write its own config into the Unreal project folder with `ModelContextProtocol.GenerateClientConfig ClaudeCode`.
We only need that if Claude Code is started inside the Unreal project folder.

### Security

- The server only listens on loopback (127.0.0.1) and rejects non-loopback origins.
- It has **no authentication**: any local program can drive the editor while it runs.
- Epic marks it as experimental. Keep the project under version control and start with read-only calls before letting Claude change the scene.

## 4. Fallback: community MCP server (UE 5.3+, also 5.7)

If Epic's plugin does not fit, for example on UE 5.7 for URLab, `sam-david/unreal-mcp` uses Unreal's built-in Python and Remote Control plugins.
It also offers Python execution, console commands, viewport screenshots and actor tools.

1. Install Node.js 18+.
2. `git clone https://github.com/sam-david/unreal-mcp`, then `npm install` and `npm run build`.
3. In Unreal, enable **Python Editor Script Plugin**, then **Enable Remote Execution** in the project settings.
4. Optional: enable **Remote Control API** for the Remote Control tools.
5. Register it with Claude Code: `claude mcp add --transport stdio unreal-mcp -- node <path>/unreal-mcp/dist/bin.js`.

## 5. Computer use

Computer use means Claude controls the screen with mouse and keyboard.
It is a fallback for things the MCP tools cannot do, like clicking through editor dialogs or visually checking the viewport.

Not verified yet: a third-party blog post says the Claude Code CLI's built-in computer-use server is macOS-only, and that the Claude Desktop app also offers computer use on Windows.
Check the current Claude Code and Claude Desktop documentation before relying on it.

For checking the scene visually, a viewport screenshot through MCP is usually more reliable than computer use.
The community server in section 4 has a screenshot tool; Epic's documentation does not mention one.

## 6. Next steps

- [ ] Finish the UE 5.8 install and create the project (C++ or Blueprint, see section 3).
- [ ] Enable Unreal MCP and connect Claude Code (section 3).
- [ ] Decide whether URLab is needed now. If yes, install Visual Studio 2022 and CMake, and decide between UE 5.7 and testing on 5.8.
- [ ] Build the arm as an MJCF model from `arm-dimensions.md`. It is engine independent, works in MuJoCo standalone and URLab, and can be checked against the worked example in `pick-and-place-plan.md`.
