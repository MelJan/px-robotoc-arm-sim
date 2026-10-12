# Web digital twin

Interactive, read-only browser twin of the measured PX robot arm. It uses the
same S1–S6 channel names, reference limits, HOME/PARK counts and provisional
tick-to-angle calibration documented in this repository.

It does not connect to or command the physical arm.

## Run

```sh
cd web
npm install
npm run dev
```

Open the URL printed by Vite, normally <http://localhost:5173>.

## Build

```sh
npm run build
```

The production site is written to `web/dist/`.

## Data

`public/arm-spec.json` is the browser-readable provisional specification. Once
`spec/arm.toml` becomes the repository's canonical source, generate this JSON
from the TOML during the build instead of maintaining both by hand.

