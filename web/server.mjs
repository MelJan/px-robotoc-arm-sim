#!/usr/bin/env node
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { connectionInfo, proxyDecision } from './src/operational/gateway.js';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const STATIC_ROOT = path.join(ROOT, 'dist');
const TYPES = {
  '.css': 'text/css; charset=utf-8',
  '.html': 'text/html; charset=utf-8',
  '.ico': 'image/x-icon',
  '.js': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.png': 'image/png',
  '.svg': 'image/svg+xml',
  '.wasm': 'application/wasm',
};

function sendJson(response, status, data) {
  response.writeHead(status, {
    'Content-Type': 'application/json; charset=utf-8',
    'Cache-Control': 'no-store',
    'X-Content-Type-Options': 'nosniff',
  });
  response.end(JSON.stringify(data));
}

function readBody(request) {
  return new Promise((resolve, reject) => {
    const chunks = [];
    let length = 0;
    request.on('data', (chunk) => {
      length += chunk.length;
      if (length > 8192) {
        reject(new Error('Request body exceeds 8192 bytes'));
        request.destroy();
      } else chunks.push(chunk);
    });
    request.on('end', () => resolve(Buffer.concat(chunks)));
    request.on('error', reject);
  });
}

function staticFile(pathname) {
  const relative = pathname === '/' ? 'index.html' : decodeURIComponent(pathname).replace(/^\/+/, '');
  const file = path.resolve(STATIC_ROOT, relative);
  return file === STATIC_ROOT || file.startsWith(`${STATIC_ROOT}${path.sep}`) ? file : null;
}

function serveStatic(request, response, pathname) {
  const file = staticFile(pathname);
  if (!file || !fs.existsSync(file) || !fs.statSync(file).isFile()) {
    sendJson(response, 404, { error: 'Unknown path. Run npm run build before starting the gateway.' });
    return;
  }
  const headers = {
    'Content-Type': TYPES[path.extname(file).toLowerCase()] || 'application/octet-stream',
    'Content-Length': fs.statSync(file).size,
    'Cache-Control': 'no-cache',
    'X-Content-Type-Options': 'nosniff',
  };
  response.writeHead(200, headers);
  if (request.method === 'HEAD') response.end();
  else fs.createReadStream(file).on('error', () => response.destroy()).pipe(response);
}

async function proxy(request, response, controller, allowMotion, pathname) {
  const decision = proxyDecision(request.method, pathname, { allowMotion });
  if (!decision.allowed) return sendJson(response, decision.status, { error: decision.error });
  if (!controller) return sendJson(response, 503, { error: 'No controller configured. Restart with --controller http://<robot>:8765.' });
  const target = new URL(`${pathname}${new URL(request.url, 'http://gateway').search}`, controller);
  const body = request.method === 'POST' ? await readBody(request) : null;
  const upstream = http.request(target, {
    method: request.method,
    headers: {
      Host: target.host,
      ...(body ? {
        Origin: `${target.protocol}//${target.host}`,
        'Content-Type': 'application/json',
        'Content-Length': body.length,
      } : {}),
    },
    timeout: 5000,
  }, (upstreamResponse) => {
    response.writeHead(upstreamResponse.statusCode ?? 502, {
      'Content-Type': upstreamResponse.headers['content-type'] || 'application/json',
      'Cache-Control': 'no-store',
      'X-Content-Type-Options': 'nosniff',
    });
    upstreamResponse.pipe(response);
  });
  upstream.on('timeout', () => upstream.destroy(new Error('controller timed out')));
  upstream.on('error', (error) => {
    if (!response.headersSent) sendJson(response, 502, { error: `Controller connection failed: ${error.message}` });
    else response.destroy();
  });
  response.on('close', () => upstream.destroy());
  upstream.end(body ?? undefined);
}

export function createGateway({ controller = null, allowMotion = false } = {}) {
  return http.createServer(async (request, response) => {
    try {
      const url = new URL(request.url, 'http://gateway');
      if (url.pathname === '/connection-info' && request.method === 'GET') {
        return sendJson(response, 200, connectionInfo(controller, allowMotion));
      }
      if (url.pathname.startsWith('/api/')) return await proxy(request, response, controller, allowMotion, url.pathname);
      if (!['GET', 'HEAD'].includes(request.method)) return sendJson(response, 405, { error: 'GET or HEAD required' });
      serveStatic(request, response, url.pathname);
    } catch (error) {
      if (!response.headersSent) sendJson(response, 500, { error: error.message });
      else response.destroy();
    }
  });
}

function option(args, name, fallback = null) {
  const index = args.indexOf(name);
  return index >= 0 ? args[index + 1] : fallback;
}

if (path.resolve(process.argv[1] ?? '') === fileURLToPath(import.meta.url)) {
  const args = process.argv.slice(2);
  const host = option(args, '--host', '127.0.0.1');
  const port = Number(option(args, '--port', '8770'));
  const rawController = option(args, '--controller');
  const allowMotion = args.includes('--allow-motion');
  let controller = null;
  if (rawController) {
    controller = new URL(rawController);
    if (controller.protocol !== 'http:' || controller.username || controller.password || controller.pathname !== '/') {
      throw new Error('--controller must be an http:// host URL without credentials or a path');
    }
  }
  if (allowMotion && !controller) throw new Error('--allow-motion requires --controller');
  const server = createGateway({ controller, allowMotion });
  server.listen(port, host, () => {
    console.log(`PX simulator gateway: http://${host}:${port}/`);
    console.log(controller ? `Controller: ${controller.origin} (${allowMotion ? 'MOTION ENABLED' : 'read-only plus stop'})` : 'Controller: none (simulation only)');
  });
}
