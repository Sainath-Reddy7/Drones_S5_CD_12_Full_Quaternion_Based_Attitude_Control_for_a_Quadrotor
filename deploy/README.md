# Static deployment (Vercel)

This folder holds the repo's zero-dependency browser simulator — a faithful
implementation of the paper's equations (1)–(21) in a single HTML file — as
`index.html`, ready to serve statically.

## Deploy (no GitHub integration needed)

The Vercel GitHub App can only be installed by the repository owner; if that's
not you, deploy straight from this clone with the CLI instead:

```bash
npm i -g vercel        # once
cd deploy
vercel deploy --prod   # first run asks you to log in via browser
```

The first deploy prints a live URL (e.g. `https://<project>.vercel.app`).
Later updates: `vercel deploy --prod` again.

## Alternative: Git-connected deploys

If the repo owner installs the Vercel GitHub App
(https://vercel.com/docs/deployments/git#installing-the-github-app) and adds
you to the Vercel project, Vercel can auto-deploy on push. For that route set
the project's **Root Directory** to `deploy/`.

## Notes

- The 6-DOF browser ground station (`pysitl/gcs/`) is NOT deployable here —
  it needs the Python SSE backend (`python -m pysitl.run --gcs`), which static
  hosting cannot run.
- Keep this folder a single self-contained `index.html`; no build step is
  used, so any static host (Vercel/Netlify/GitHub Pages) serves it unchanged.
