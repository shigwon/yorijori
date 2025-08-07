<template>
    <div class="line-chart">
      <VueApexCharts
        :options="chartOptions"
        :series="series"
        :height="height"
      />
    </div>
  </template>
  
  <script setup>
  import { computed } from 'vue'
  import VueApexCharts from 'vue3-apexcharts'
  import { defineProps } from 'vue'
  
  const props = defineProps({
    data: {
      type: Array,
      default: () => []
    },
    categories: {
      type: Array,
      default: () => []
    },
    colors: {
      type: Array,
      default: () => ['#1e40af']
    },
    height: {
      type: [String, Number],
      default: 300
    }
  })
  
  const series = computed(() => {
    return [{
      name: 'Orders',
      data: props.data
    }]
  })
  
  const chartOptions = computed(() => {
    return {
      chart: {
        type: 'line',
        toolbar: {
          show: false
        },
        background: 'transparent'
      },
      stroke: {
        curve: 'smooth',
        width: 5,
        colors: props.colors
      },
      fill: {
        type: 'gradient',
        gradient: {
          shade: 'dark',
          type: 'vertical',
          shadeIntensity: 0.2,
          gradientToColors: props.colors,
          inverseColors: false,
          opacityFrom: 0.5,
          opacityTo: 0.2,
          stops: [0, 100]
        }
      },
      dataLabels: {
        enabled: false
      },
      xaxis: {
        categories: props.categories,
        labels: {
          style: {
            colors: '#8a92a6',
            fontSize: '12px'
          }
        },
        axisBorder: {
          show: false
        },
        axisTicks: {
          show: false
        }
      },
      yaxis: {
        labels: {
          style: {
            colors: '#8a92a6',
            fontSize: '12px'
          },
          formatter: function (val) {
            return val.toFixed(0)
          }
        },
        min: 0,
        max: 40,
        tickAmount: 5,
        forceNiceScale: true
      },
      tooltip: {
        theme: 'dark',
        y: {
          formatter: function (val) {
            return val.toFixed(0)
          }
        }
      },
      grid: {
        borderColor: '#374151',
        strokeDashArray: 5,
        xaxis: {
          lines: {
            show: false
          }
        },
        yaxis: {
          lines: {
            show: true,
            color: '#4b5563'
          }
        }
      },
      theme: {
        mode: 'dark'
      },
      markers: {
        size: 7,
        colors: props.colors,
        strokeColors: '#ffffff',
        strokeWidth: 2,
        hover: {
          size: 9
        }
      }
    }
  })
  </script>
  
  <style scoped>
  .line-chart {
    width: 100%;
    height: 100%;
  }
  </style> 