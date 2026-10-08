# supervisor_manager

## Overview

`supervisor_manager` is the high-level coordination package responsible for orchestrating supervisory modules within the robotic architecture.

It integrates:

- `supervisor_system`
- `supervisor_plan`
- (Future supervisory components)

Its objective is to ensure global coherence between system operability and plan-level supervision.

---

## Architectural Role

Within the layered architecture:

Planner → Supervisor → Executor

`supervisor_manager` acts as the supervisory coordinator, ensuring that:

- The system is operational.
- Plans are coherent.
- Corrective strategies are applied when necessary.

---

## Functional Responsibilities

### 1. System-State Verification

The manager queries `supervisor_system` to determine:

- Whether required modules are active.
- Whether recovery actions are required.

If the system is unstable, corrective measures are initiated.

---

### 2. Plan Coordination

The manager supervises `supervisor_plan` and can:

- Trigger plan corrections.
- Inject corrective instructions.
- Request re-evaluation of current plan states.

This allows cross-layer corrective behavior.

---

### 3. Cross-Supervisor Arbitration

The manager serves as a mediator between:

- System-level supervision
- Plan-level supervision

If system instability affects plan feasibility, the manager can:

- Request plan adjustment.
- Inject fallback instructions.
- Delay execution until stability is restored.

---

## Design Objectives

- Hierarchical supervisory control.
- Decoupled module coordination.
- Runtime adaptability.
- Structural modularity.
- Scalability for additional supervisors.

---

## Dependencies

- ROS 2 (rclcpp)
- supervisor_system
- supervisor_plan

---

## Future Work

- Global mission-level monitoring.
- Supervisor priority arbitration.

---

## Author: Joshua Martinez

Bio-Robotics Laboratory  
Universidad Nacional Autónoma de México


## Piloto de recuperación de navegación

La política está en `include/supervisor_manager/navigation_recovery.hpp` y
`src/navigation_recovery.cpp`. No publica comandos ROS: devuelve una decisión
al manager. La configuración es `config/navigation_recovery.yaml`.

Casos incluidos:

| Código (`failure_reason`) | Comportamiento predeterminado |
| --- | --- |
| `location_not_found` | Pedir otra ubicación, hasta dos capturas |
| `navigation_failed` | Reintentar hasta tres veces |
| `arrival_outside_tolerance` | Reintentar hasta dos veces |
| `bumper_triggered` | Terminar la tarea |

`max_attempts` cuenta capturas nuevas o reintentos adicionales; la ejecución
original no cuenta. Los presupuestos se conservan durante una recuperación y
se reinician cuando la navegación termina correctamente o empieza otro plan.
Al agotarse un presupuesto se anuncia `exhausted_phrase` y se termina la tarea.

### Cambiar una frase o política

```yaml
cases:
  location_not_found:
    behavior: ask_location
    max_attempts: 2
    phrase: "No conozco {argument}. Dime otra ubicación."
    invalid_phrase: "Esa ubicación no es conocida. Dime otra."
```

Edita el archivo completo, conservando los cuatro casos requeridos.
`behavior` acepta `ask_location`, `retry` o `abort`. Puedes añadir otro código
que reporte una acción `go_to`; se aplica con el mismo mecanismo.
`{argument}` se sustituye por el destino actual. Las frases deben ser compatibles
con el idioma configurado en el listener y el talker.

Para usar una copia de configuración propia:

```bash
ros2 run supervisor_manager supervisor_manager_node --ros-args \
  -p navigation_recovery_config:=/ruta/navigation_recovery.yaml
```

La configuración se valida al arrancar y requiere reiniciar el nodo para
recargarla. Un archivo inválido impide el arranque; no se aplica parcialmente.
Este piloto configura casos y comportamientos conocidos; todavía no implementa
un lenguaje general para secuencias arbitrarias de pasos.

### Ubicación corregida

La escucha se realiza de forma asíncrona con un tiempo límite configurable
(`service_timeout_sec`). El nombre se normaliza (espacios y guiones a `_`,
minúsculas) y se consulta mediante `locations_manager/get`, sin iniciar movimiento.
Solo `success=true` permite insertar el nuevo `go_to`. La recuperación no valida
contra `knowledge_bank/domains/locations.yaml`: ese catálogo se conserva para
el pipeline semántico GPSR y no garantiza que exista una pose navegable.

El servicio se configura mediante `navigation_locations_service`, cuyo valor
predeterminado es `/locations_manager/get`. Si navegación usa un namespace,
configura la misma ruta que usa su controlador. Los logs `VALIDATE` muestran la
ruta, el nombre consultado y la respuesta. No se aceptan nombres compactos ni
alias que no estén registrados en navegación.

Una respuesta desconocida, silencio, fallo o timeout del servicio consume un
intento y vuelve a pedir la ubicación. Nunca se inserta un destino sin confirmar
su existencia. La consulta usa el catálogo actualmente cargado por navegación,
incluidas las ubicaciones guardadas durante la ejecución.

El destino corregido se ejecuta como sustitución del paso fallido y queda como
última instrucción enviada por el plan. Por eso los siguientes reintentos usan
el destino nuevo. El cursor del plan original no se rebobina y los pasos futuros
no se modifican: cambiar el destino de toda la tarea queda fuera de este piloto.

### Coordinación y contratos corregidos

El manager acumula las operaciones de plan y llama a
`/supervisor/supervisor_plan/prepare_cycle` (`clips_interfaces/srv/String`).
La solicitud lleva un arreglo JSON con operaciones `command`, `override` o
`rewind`. El plan valida el arreglo completo, aplica las operaciones en orden e
instala el permiso de despacho. Solo después de recibir `ready` el manager
publica el pulso de ejecución. Se elimina la espera fija de 50 ms.

Si falta el servicio o el suscriptor de ejecución, el despacho queda pendiente.
Si la respuesta del servicio tarda más de diez segundos, se termina la tarea:
no se reenvían operaciones de efecto ambiguo, como un salto o rebobinado.
Manager y plan deben actualizarse juntos; se conservan los tópicos antiguos en
el plan para consumidores existentes, pero el manager nuevo usa el servicio.

El reporte se conserva hasta recibir el estado de plan necesario para decidir.
`failure_reason` proporciona códigos estables de navegación y objeto ausente;
no se modificaron las interfaces ROS. Los mensajes del worker siguen disponibles
para diagnóstico y compatibilidad.

También se corrigieron la prioridad de las correcciones de escucha, el estado
predeterminado de un fallo en el validador y la selección de recuperación de
`take`: PNP solo interviene en modo directo y para sus objetos administrados.
Se agregaron contratos de validación para `give`, `receive`, `find_space` y
`analyze_person`. Este último emite `person_analyzed:` en éxito.

Los fallos de entrega/recepción posteriores a efectos físicos que no pueden
confirmarse se clasifican como `physical_effect_uncertain` y detienen la tarea.
No se reintenta automáticamente una entrega o recepción parcial. Las interfaces
actuales aún no transmiten fases detalladas, estado físico confirmado ni un
identificador de ejecución en el reporte del supervisor; esa ampliación queda
para los siguientes módulos.

El pipeline GPSR utiliza la API actual `goalsToClips()` y exige metas JSON. La
API antigua `primitiveToClips()` fue retirada de `planning_tools`.

### Verificación sin hardware

```bash
colcon build --packages-select planning_tools executor_tools \
  supervisor_plan supervisor_execution supervisor_manager
source install/setup.bash
colcon test --packages-select supervisor_manager supervisor_execution \
  --ctest-args -R 'test_(navigation_recovery|pnp_recovery|recovery_contracts)'
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 \
  src/Supervisor/supervisor_manager/test/navigation_pilot_smoke.py
```

La prueba integrada utiliza manager y plan reales con planificación, ejecución
y HRI simulados en un dominio ROS separado (173, configurable con
`ROS_DOMAIN_ID`). No inicia drivers ni mueve el robot. Comprueba captura inválida,
destino corregido, reintento con ese destino, ejecución que se conecta tarde,
estado de plan que llega después del resultado y conservación del siguiente
paso original. Requiere sockets DDS locales disponibles.

Para verificar agotamiento sin avanzar el plan, ejecuta también la prueba con
`--exhausted`.


## Logs de recuperación y continuación para depuración

Los logs INFO muestran `[NavigationRecovery] BEGIN`, el código de error, el
comportamiento elegido y cada transición de estado. `auxiliary SAY` y
`auxiliary LISTEN` son llamadas directas a HRI, no instrucciones insertadas en
el plan. `[PlanMutation]` muestra instrucciones insertadas y comandos de
reintento/salto; `[PlanDispatch]` muestra el lote enviado y su confirmación.
`CURRENT_INSTRUCTION` sigue mostrando lo efectivamente despachado por el plan.
Los resultados de escucha, destinos rechazados y fallos/timeouts de voz también
se registran.

`continue_on_recovery_failure` es un parámetro ROS booleano, false por defecto:

```bash
# Depuración: tras agotar la recuperación de navegación, omitir el paso fallido.
ros2 run supervisor_manager supervisor_manager_node --ros-args \
  -p continue_on_recovery_failure:=true

# Detener cuando la recuperación no consigue resolver el fallo.
ros2 run supervisor_manager supervisor_manager_node --ros-args \
  -p continue_on_recovery_failure:=false
```

También puede cambiarse con `ros2 param set` durante la ejecución; la decisión
lee el valor actual al terminar la recuperación. Se aplica al agotamiento/aborto
de navegación y al manejo genérico de acciones fallidas. No desactiva los
reintentos. Con true se anuncia el fallo y se continúa solo si quedan pasos;
con false se termina. Los casos con políticas especializadas (PNP, puerta,
captura de datos) conservan sus decisiones propias. El bumper, los efectos
físicos inciertos y los errores del protocolo de despacho siempre detienen la
tarea. El archivo YAML sigue controlando frases y presupuestos; al continuar
por depuración se sustituye el anuncio final de detenerse por uno de continuar.

Pruebas integradas de ambos modos:

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 \
  src/Supervisor/supervisor_manager/test/navigation_pilot_smoke.py --exhausted
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 \
  src/Supervisor/supervisor_manager/test/navigation_pilot_smoke.py \
  --exhausted --continue-on-failure
```


## Fuente de ubicaciones navegables

La recuperación consulta el servicio existente de `locations_manager`, sin
modificar ese paquete ni leer directamente su archivo. El catálogo que dicho
nodo tenga cargado es la fuente de verdad. `navigation_locations_service`
permite elegir la misma ruta de servicio que usa el controlador de navegación.
La validación y carga del archivo de coordenadas quedan a cargo de ese paquete.

Prueba integrada con `locations_manager` real, sin hardware:

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 \
  src/Supervisor/supervisor_manager/test/navigation_pilot_smoke.py --real-locations
```

Comprueba que `kitchen` (solo semántico) sea rechazado y `cooking_table`
(con pose navegable, ausente del listado semántico) sea aceptado.


## Recuperación de manipulación: take fuera de alcance

`ManipulationRecovery` mantiene el estado y el presupuesto de la secuencia
`approach objeto -> take objeto`. La configuración está en
`config/manipulation_recovery.yaml`; se puede seleccionar otro archivo con el
parámetro `manipulation_recovery_config`. Reinicia el manager para recargarla.

El caso `take_pose_out_of_range` se activa únicamente al fallar `take` con
`pose_out_of_range`. La secuencia y su estado están fuera del manager; este
aplica los comandos devueltos por la clase y anuncia las frases mediante HRI.
Los logs `[ManipulationRecovery]` muestran estados y resultados; `[PlanMutation]`
muestra cada instrucción insertada. La llamada de voz del adaptador todavía es
síncrona, como otras frases del manager.

```yaml
cases:
  take_pose_out_of_range:
    enabled: true
    max_attempts: 1
    phrase: "Necesito acercarme antes de intentar agarrar el objeto."
    approach_instruction: "Justina approach {argument}"
    take_instruction: "Justina take {argument}"
    failed_phrase: "No pude recuperar el agarre."
```

`max_attempts` cuenta ciclos adicionales completos. Si el acercamiento falla,
la secuencia termina sin intentar take. Si el take vuelve a fallar por alcance,
puede iniciar otro ciclo mientras quede presupuesto. Otro error del take termina
la recuperación. El agotamiento respeta `continue_on_recovery_failure`; bumper
siempre detiene. El éxito vuelve a la evaluación normal del reporte y conserva
la verificación `verify_take` de PNP cuando corresponda. No se consume ningún
paso adicional del plan original para insertar la secuencia.

Por ahora las plantillas de instrucciones admitidas son las dos mostradas,
con el mismo objeto. Las frases, enabled y max_attempts son configurables;
el YAML no permite inventar acciones ni secuencias arbitrarias. La recuperación
PNP ya activa conserva la propiedad de sus instrucciones y su política propia.
Los reintentos generales de take por otros motivos siguen en el manager.

Prueba sin hardware (plan y manager reales; ejecutor/HRI simulados):

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 \
  src/Supervisor/supervisor_manager/test/navigation_pilot_smoke.py --manipulation
# Agrega --fail-recovery approach o --fail-recovery take para simular un fallo.
# Agrega --continue-on-failure para comprobar que se retoma el plan original.
```

Pendiente de la separación completa: sacar del manager la ejecución de
navegación (voz, escucha y consulta), completar drop y verificación de agarre en
ManipulationRecovery, y extraer percepción, personas, interacción y políticas de
seguimiento/GPSR. También faltan herramientas generales para editar secuencias
del plan y reportes con identificador de ejecución/fase física.
