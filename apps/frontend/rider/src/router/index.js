import { createRouter, createWebHistory } from 'vue-router'

// 컴포넌트들을 동적 import로 로드
const HowToUseScreen = () => import('../components/01_HowToUseScreen.vue')
const ScanOptionScreen = () => import('../components/02_ScanOptionScreen.vue')
const ReceiptScanScreen = () => import('../components/03_ReceiptScanScreen.vue')
const ManualInputScreen = () => import('../components/04_ManualInputScreen.vue')
const ManualConfirmScreen = () => import('../components/05_ManualConfirmScreen.vue')
const LocationRequestScreen = () => import('../components/06_LocationRequestScreen.vue')
const PhotoCaptureScreen = () => import('../components/07_PhotoCaptureScreen.vue')
const CompleteScreen = () => import('../components/08_CompleteScreen.vue')


const routes = [
  {
    path: '/rider/',
    redirect: '/rider/how-to-use'
  },
  {
    path: '/rider/how-to-use',
    name: 'HowToUse',
    component: HowToUseScreen,
    meta: { progress: 10 }
  },
  {
    path: '/rider/scan-option',
    name: 'ScanOption',
    component: ScanOptionScreen,
    meta: { progress: 20 }
  },
  {
    path: '/rider/receipt-scan',
    name: 'ReceiptScan',
    component: ReceiptScanScreen,
    meta: { progress: 30 }
  },
  {
    path: '/rider/manual-input',
    name: 'ManualInput',
    component: ManualInputScreen,
    meta: { progress: 40 }
  },
  {
    path: '/rider/manual-confirm',
    name: 'ManualConfirm',
    component: ManualConfirmScreen,
    meta: { progress: 50 }
  },
  {
    path: '/rider/location-request',
    name: 'LocationRequest',
    component: LocationRequestScreen,
    meta: { progress: 70 }
  },
  {
    path: '/rider/photo-capture',
    name: 'PhotoCapture',
    component: PhotoCaptureScreen,
    meta: { progress: 80 }
  },
  {
    path: '/rider/complete',
    name: 'Complete',
    component: CompleteScreen,
    meta: { progress: 100 }
  },

]

const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router
