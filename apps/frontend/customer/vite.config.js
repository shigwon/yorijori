import { fileURLToPath, URL } from 'node:url'

import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import vueDevTools from 'vite-plugin-vue-devtools'

// https://vite.dev/config/
export default defineConfig({
  base: '/',
  plugins: [
    vue(),
    vueDevTools(),
  ],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url))
    },
  },
  server: {
    host: '0.0.0.0',  // 외부 접속 허용
    port: 5173,
    allowedHosts: ['.ngrok-free.app'],  // ✅ ngrok 주소 허용
    proxy: {
      '/api': {
        target: 'http://192.168.100.82:8080', // 백엔드 서버 IP
        changeOrigin: true,
        rewrite: (path) => path,
      },
    },
  }
})