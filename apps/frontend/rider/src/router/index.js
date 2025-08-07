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
    path: '/',
    redirect: '/how-to-use'
  },
  {
    path: '/how-to-use',
    name: 'HowToUse',
    component: HowToUseScreen,
    meta: { progress: 10 }
  },
  {
    path: '/scan-option',
    name: 'ScanOption',
    component: ScanOptionScreen,
    meta: { progress: 20 }
  },
  {
    path: '/receipt-scan',
    name: 'ReceiptScan',
    component: ReceiptScanScreen,
    meta: { progress: 30 }
  },
  {
    path: '/manual-input',
    name: 'ManualInput',
    component: ManualInputScreen,
    meta: { progress: 40 }
  },
  {
    path: '/manual-confirm',
    name: 'ManualConfirm',
    component: ManualConfirmScreen,
    meta: { progress: 50 }
  },
  {
    path: '/location-request',
    name: 'LocationRequest',
    component: LocationRequestScreen,
    meta: { progress: 70 }
  },
  {
    path: '/photo-capture',
    name: 'PhotoCapture',
    component: PhotoCaptureScreen,
    meta: { progress: 80 }
  },
  {
    path: '/complete',
    name: 'Complete',
    component: CompleteScreen,
    meta: { progress: 100 }
  },

]

const router = createRouter({
  history: createWebHistory('/rider/'),
  routes
})

export default router
