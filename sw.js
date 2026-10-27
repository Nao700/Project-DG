// オフラインでも開けるようにアプリ本体をキャッシュする。
// ファイルを更新したら VERSION を上げること（古いキャッシュが消える）
const VERSION = 'coyote-v9';
const FILES = ['./', './index.html', './manifest.webmanifest', './icon.svg', './icon-192.png', './icon-512.png'];

self.addEventListener('install', e => {
  e.waitUntil(caches.open(VERSION).then(c => c.addAll(FILES)).then(() => self.skipWaiting()));
});
self.addEventListener('activate', e => {
  e.waitUntil(caches.keys()
    .then(keys => Promise.all(keys.filter(k => k !== VERSION).map(k => caches.delete(k))))
    .then(() => self.clients.claim()));
});
// ネットワーク優先。つながらないときだけキャッシュを使う（更新がすぐ反映されるように）
self.addEventListener('fetch', e => {
  // 自分のサイトのファイルだけ扱う（AI の API などはキャッシュしない）
  if (e.request.method !== 'GET' || new URL(e.request.url).origin !== self.location.origin) return;
  e.respondWith(
    fetch(e.request)
      .then(res => {
        const copy = res.clone();
        caches.open(VERSION).then(c => c.put(e.request, copy)).catch(() => {});
        return res;
      })
      .catch(() => caches.match(e.request).then(r => r || caches.match('./index.html')))
  );
});
