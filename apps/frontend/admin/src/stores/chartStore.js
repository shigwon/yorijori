import { defineStore } from 'pinia'
import { ref } from 'vue'

export const useChartStore = defineStore('chart', () => {
  // 상태
  const selectedDate = ref(new Date().toISOString().split('T')[0]) // 오늘 날짜 기본값
  const hourlyData = ref([])
  const isLoading = ref(false)
  const error = ref(null)

  // 액션
  const setSelectedDate = (date) => {
    selectedDate.value = date
  }

  const setHourlyData = (data) => {
    hourlyData.value = data
  }

  const setLoading = (loading) => {
    isLoading.value = loading
  }

  const setError = (err) => {
    error.value = err
  }

  const clearError = () => {
    error.value = null
  }

  return {
    // 상태
    selectedDate,
    hourlyData,
    isLoading,
    error,
    
    // 액션
    setSelectedDate,
    setHourlyData,
    setLoading,
    setError,
    clearError
  }
}) 