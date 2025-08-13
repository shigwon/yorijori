import axios from 'axios'

// 일반 API 요청을 위한 axios instance
const apiClient = axios.create({
  baseURL: '/api',
  timeout: 1000,
  headers: {
    'Content-Type': 'application/json',
  }
})

// 요청 인터셉터
apiClient.interceptors.request.use(
  (config) => {
    // 토큰이 있으면 헤더에 추가
    const token = localStorage.getItem('adminToken')
    if (token) {
      config.headers.Authorization = `Bearer ${token}`
    }
    
    console.log('API 요청:', config.method?.toUpperCase(), config.url)
    return config
  },
  (error) => {
    console.error('API 요청 오류:', error)
    return Promise.reject(error)
  }
)

// 응답 인터셉터
apiClient.interceptors.response.use(
  (response) => {
    console.log('API 응답:', response.status, response.config.url)
    return response
  },
  (error) => {
    console.error('API 응답 오류:', error.response?.status, error.response?.data)
    
    // 401 Unauthorized 처리
    if (error.response?.status === 401) {
      localStorage.removeItem('adminToken')
      window.location.href = '/'
    }
    
    return Promise.reject(error)
  }
)

export default apiClient 