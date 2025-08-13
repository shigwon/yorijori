import { fileURLToPath, URL } from 'node:url'

import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import vueDevTools from 'vite-plugin-vue-devtools'

// https://vite.dev/config/
export default defineConfig({
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
    port: 3000,
    proxy: {
      '/api/v1/streaming/':{                     // webrtc 발급용 api
        target: 'http://192.168.100.83:8080',
        changeOrigin: true,
        secure: false
      },
      '/api': {
        target: 'http://192.168.100.115:8080', // 백엔드 서버 주소
        changeOrigin: true,
        secure: false,
        ws: true, // 웹소켓 프록시 설정
      },
      
    }
  }
})
