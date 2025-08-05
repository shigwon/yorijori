import { ref } from 'vue'

const progressPercent = ref(0)

function setProgressPercent(percent) {
  progressPercent.value = percent
}

const currentScreen = ref('how-to-use')
const receiptData = ref({ id: '', tel: '' })
const orderData = ref({ orderNumber: '', safeNumber: '' })

const goToScanOption = () => {
  currentScreen.value = 'scan-option'
  console.log('화면 전환:', currentScreen.value)
}

const goToReceiptScan = () => {
  currentScreen.value = 'receipt-scan'
  console.log('화면 전환:', currentScreen.value)
}

const goToManualInput = () => {
  currentScreen.value = 'manual-input'
  console.log('화면 전환:', currentScreen.value)
}

const goToManualConfirm = (orderNumber, safeNumber) => {
  orderData.value = { orderNumber, safeNumber }
  currentScreen.value = 'manual-confirm'
  console.log('화면 전환:', currentScreen.value)
}

const goToLocationRequest = () => {
  currentScreen.value = 'location-request'
  console.log('화면 전환:', currentScreen.value)
}

const goToLoadingModal = () => {
  currentScreen.value = 'loading-modal'
  console.log('화면 전환:', currentScreen.value)
}

const goToScanConfirmModal = () => {
  currentScreen.value = 'scan-confirm-modal'
  console.log('화면 전환:', currentScreen.value)
}

export const useAppState = () => {
  const goToHowToUse = () => {
    currentScreen.value = 'how-to-use'
    console.log('화면 전환:', currentScreen.value)
  }

  return {
    currentScreen,
    receiptData,
    orderData,
    goToHowToUse,
    goToScanOption,
    goToReceiptScan,
    goToManualInput,
    goToManualConfirm,
    goToLocationRequest,
    goToLoadingModal,
    goToScanConfirmModal,
    progressPercent,
    setProgressPercent,
  }
}