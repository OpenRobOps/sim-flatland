# ISO 21423 agent for flatland. The SDK (@openrobops/iso21423) is not published yet, so it is
# built from a second build context named `iso21423` (the SDK repo root):
#   docker compose build iso-agent            # compose passes ../../iso21423 (sibling checkout)
#   docker build --build-context iso21423=/path/to/iso21423 iso-agent
FROM node:22-slim
WORKDIR /iso21423/ts-sdk
COPY --from=iso21423 ts-sdk/package.json ts-sdk/package-lock.json ts-sdk/tsconfig.json ts-sdk/tsup.config.ts ./
COPY --from=iso21423 ts-sdk/src ./src
RUN npm ci && npm run build && npm prune --omit=dev
WORKDIR /app
COPY package.json ./
# package.json points at the sibling checkout for local dev; inside the image the SDK lives at /iso21423.
# --install-links COPIES the file: dependency under /app/node_modules instead of symlinking it, so the
# SDK's lazy `import('mqtt')` resolves against /app's node_modules (the peer dependency lives there).
# A symlinked SDK resolves from its real path (/iso21423/ts-sdk), where `npm prune --omit=dev` above
# has removed mqtt — that was the "Cannot find package 'mqtt'" startup crash.
RUN sed -i 's|file:../../iso21423/ts-sdk|file:/iso21423/ts-sdk|' package.json && npm install --omit=dev --install-links
COPY src ./src
CMD ["node", "src/main.js"]
